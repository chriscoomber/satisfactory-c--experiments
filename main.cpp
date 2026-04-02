#include "Item.h"
#include "Recipe.h"
#include "nlohmann/json_fwd.hpp"
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

	return 0;
}