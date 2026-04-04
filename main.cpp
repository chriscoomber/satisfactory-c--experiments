#include "Item.h"
#include "Recipe.h"
#include "nlohmann/json_fwd.hpp"
#include "simplex.h"
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

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
			recipes[recipe.id] = recipe;
		}
	}

	// Test
	std::cout << items["Desc_IronRod_C"] << "\n";
	std::cout << recipes["Recipe_IronPlateReinforced_C"] << "\n";

	const std::size_t N{ 4 };
	const std::size_t M{ 3 };

	std::array<std::array<double, N>, M> A{ };
	std::array<double, M> b{  };
	std::array<double, N> c{ };
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

	auto x = simplex<M, N>(A, b, c, minimize);
	if (!x) {
		std::cout << "Infeasible!";
	}
	else {
		for (std::size_t i{ 0 }; i < x->size(); ++i) {
			std::cout << (*x)[i] << " ";
		}
	}

	return 0;
}