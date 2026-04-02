#pragma once

#include "nlohmann/json_fwd.hpp"
#include <nlohmann/json.hpp>

#include <iostream>
#include <string>
#include <vector>
using json = nlohmann::json;

struct Recipe {
	struct Ingredient {
		std::string itemId;
		int amount;
	};

	std::string id;
	std::string displayName;
	std::vector<Ingredient> ingredients;
	std::vector<Ingredient> products;
	double duration; // in seconds
	bool sloopable;
};

std::ostream& operator<<(std::ostream& os, const Recipe& recipe) {
	return os << "Recipe(" << recipe.id << ")";
}

std::ostream& operator<<(std::ostream& os, const Recipe::Ingredient ingredient) {
	return os << ingredient.itemId << "(" << ingredient.amount << ")";
}

Recipe::Ingredient ingredientFromJson(const json& j) {
	return Recipe::Ingredient{ .itemId{ j["itemId"] }, .amount{j["amount"]} };
}

Recipe recipeFromJson(const json& j) {
	std::vector<Recipe::Ingredient> ingredients{};
	for (auto& [_, value] : j["ingredients"].items()) {
		ingredients.push_back(ingredientFromJson(value));
	}

	std::vector<Recipe::Ingredient> products{};
	for (auto& [_, value] : j["products"].items()) {
		products.push_back(ingredientFromJson(value));
	}

	return Recipe{ .id{j["id"]}, .displayName{j["displayName"]}, .ingredients{ingredients}, .products{products} };
}
