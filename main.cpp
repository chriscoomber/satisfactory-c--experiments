#include "Item.h"
#include "Recipe.h"
#include "nlohmann/json_fwd.hpp"
#include "simplex.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

using json = nlohmann::json;

int main()
{
	// Load items
	std::map<std::string, Item> items{};
	{
		std::ifstream f("data/items.json");
		json data = json::parse(f);

		for (auto const& i : data) {
			auto item = itemFromJson(i);
			items[item.id] = item;
		}
	}

	// Load recipes
	std::map<std::string, Recipe> recipes{};
	{
		std::ifstream f("data/recipes.json");
		json data = json::parse(f);

		for (auto const& i : data) {
			auto recipe = recipeFromJson(i);

			// Check all the items are ones we know about
			bool isValid = true;
			for (const auto& item : recipe.ingredients) {
				if (!items.contains(item.itemId)) {
					isValid = false;
					break;
				}
			}
			if (!isValid) continue; // skip this recipe
			for (const auto& item : recipe.products) {
				if (!items.contains(item.itemId)) {
					isValid = false;
					break;
				}
			}
			if (!isValid) continue; // skip this recipe

			recipes[recipe.id] = recipe;
		}
	}

	// Test
	{
		std::cout << items["Desc_IronRod_C"] << "\n";
		std::cout << recipes["Recipe_IronPlateReinforced_C"] << "\n";

		const std::size_t N{ 4 };
		const std::size_t M{ 3 };

		std::vector<std::vector<double>> A(M, std::vector(N, 0.0));
		std::vector<double> b(M, 0.0);
		std::vector<double> c(N, 0.0);
		bool minimize = false;

		// Unbounded
		//A[0][0] = 1; A[0][1] = -1; A[0][2] = 1;
		//A[1][0] = -2; A[1][1] = 1; A[1][2] = 0;
		//A[2][0] = 0; A[2][1] = 1; A[2][2] = -2;
		//b[0] = 5; b[1] = 3; b[2] = 5;
		//c[0] = 0; c[1] = 2; c[2] = 1;

		// Infeasible
		//A[0][0] = -1; A[0][1] = 1;
		//A[1][0] = 1; A[1][1] = 1;
		//A[2][0] = 1; A[2][1] = -4;
		//b[0] = 8; b[1] = -3; b[2] = 2;
		//c[0] = 1; c[1] = 3;

		// Minimization problem, should give 0.5, 0.5, 0, 0.5
		A[0][0] = 1; A[0][1] = 0; A[0][2] = -2; A[0][3] = -1;
		A[1][0] = 0; A[1][1] = 1; A[1][2] = 0; A[1][3] = -1;
		A[2][0] = 0; A[2][1] = 0; A[2][2] = 1; A[2][3] = 2;
		b[0] = 0; b[1] = 0; b[2] = 1;
		c[0] = 1; c[1] = 1; c[2] = 1; c[3] = 1;
		minimize = true;

		auto xOrError = simplex(A, b, c, minimize);
		if (const auto error(std::get_if<SimplexError>(&xOrError)); error) {
			switch (*error) {
			case SimplexError::Infeasible:

				std::cout << "Infeasible!";
				break;
			case SimplexError::InvalidInput:
				std::cout << "Invalid!";
				break;
			}
		}
		else if (const auto x(std::get_if<std::vector<double>>(&xOrError)); x) {
			for (std::size_t i{ 0 }; i < x->size(); ++i) {
				std::cout << (*x)[i] << " ";
			}
		}

		std::cout << "\n\n";
	}

	// Right... satisfactory
	// Each item -> constraint (i.e. that the total produced by that item is positive) (length M)
	// Each recipe -> value (i.e. how much of that recipe to make, must be >= 0) (length N)
	// Goal function is just to minimize the resources on the belt. The closest I can get to this is to assign each
	// recipe a number equal to the max of its inputs and outputs.
	// (Fluids technically don't go on belts, but to avoid thinking about that more, just divide each by 1000).
	const std::size_t M{ items.size() };
	const std::size_t N{ recipes.size() };

	// Assign each recipe an index
	std::vector<std::string_view> recipeFromIndex;
	recipeFromIndex.reserve(N);
	std::transform(recipes.begin(), recipes.end(), std::back_inserter(recipeFromIndex), [](const std::pair<std::string_view, Recipe&>& p) {
		return p.first;
		});

	std::map<std::string_view, std::size_t> recipeToIndex;
	for (std::size_t n{ 0 }; n < N; ++n) {
		recipeToIndex[recipeFromIndex[n]] = n;
	}

	// Assign each item an index
	std::vector<std::string_view> itemFromIndex;
	itemFromIndex.reserve(M);
	std::transform(items.begin(), items.end(), std::back_inserter(itemFromIndex), [](const std::pair<std::string_view, Item&>& p) { return p.first; });

	std::map<std::string_view, std::size_t> itemToIndex;
	for (std::size_t m{ 0 }; m < M; ++m) {
		itemToIndex[itemFromIndex[m]] = m;
	}

	// Create the matrix of constraints A
	std::vector<std::vector<double>> A(M, std::vector(N, 0.0));

	for (const auto& [_key, recipe] : recipes) {
		const std::size_t n = recipeToIndex[recipe.id];

		for (const auto& ingredient : recipe.ingredients) {
			const std::size_t m = itemToIndex[ingredient.itemId];
			const Item& item = items[ingredient.itemId];
			const double divisor = item.form == Item::Form::solid ? 1 : 1000;
			const double amountPerSecond = (ingredient.amount / divisor) / recipe.duration;
			A[m][n] = -amountPerSecond;
		}

		for (const auto& ingredient : recipe.products) {
			const std::size_t m = itemToIndex[ingredient.itemId];
			const Item& item = items[ingredient.itemId];
			const double divisor = item.form == Item::Form::solid ? 1 : 1000;
			const double amountPerSecond = (ingredient.amount / divisor) / recipe.duration;
			A[m][n] = amountPerSecond;
		}
	}

	// Create the vector of bounds b - super easy, set most things to zero.
	std::vector<double> b(M, 0.0);
	b[itemToIndex["Desc_IronPlateReinforced_C"]] = 1.0;

	// Create the vector of goal c - for each recipe we want to minimize the max of input and output amount
	std::vector<double> c(N, 0.0);
	for (const auto& [_key, recipe] : recipes) {
		const std::size_t n = recipeToIndex[recipe.id];

		double ingredientPressure{ 0 };
		for (const auto& ingredient : recipe.ingredients) {
			const Item& item = items[ingredient.itemId];
			const double divisor = item.form == Item::Form::solid ? 1 : 1000;
			const double amountPerSecond = (ingredient.amount / divisor) / recipe.duration;
			ingredientPressure += amountPerSecond;
		}

		double productPressure{ 0 };
		for (const auto& ingredient : recipe.products) {
			const Item& item = items[ingredient.itemId];
			const double divisor = item.form == Item::Form::solid ? 1 : 1000;
			const double amountPerSecond = (ingredient.amount / divisor) / recipe.duration;
			productPressure += amountPerSecond;
		}

		c[n] = std::max(ingredientPressure, productPressure);
	}

	// Let's do it!
	auto xOrError = simplex(A, b, c, true);
	if (const auto error(std::get_if<SimplexError>(&xOrError)); error) {
		switch (*error) {
		case SimplexError::Infeasible:
			std::cout << "Infeasible!";
			break;
		case SimplexError::InvalidInput:
			std::cout << "Invalid!";
			break;
		}
	}
	else if (const auto x(std::get_if<std::vector<double>>(&xOrError)); x) {
		for (std::size_t i{ 0 }; i < x->size(); ++i) {
			if ((*x)[i] == 0.0) continue;
			std::string_view recipeId = recipeFromIndex[i];
			std::cout << recipes[std::string{ recipeFromIndex[i] }].displayName << ": " << (*x)[i] << "\n";
		}
	}


	return 0;
}