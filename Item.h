#pragma once

#include "nlohmann/json_fwd.hpp"
#include <nlohmann/json.hpp>

#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
using json = nlohmann::json;

struct Item {
	enum class Form {
		solid, liquid, gas
	};
	std::string id;
	std::string displayName;
	Form form;
};

std::ostream& operator<<(std::ostream& os, const Item& item) {
	return os << "Item(" << item.id << ")";
}

std::ostream& operator<<(std::ostream& os, const Item::Form form) {
	switch (form)
	{
	case Item::Form::solid:		return os << "Form(solid)";
	case Item::Form::liquid:	return os << "Form(liquid)";
	case Item::Form::gas:		return os << "Form(gas)";
	default: throw std::out_of_range("unexpected Item::Form");
	}
}

Item::Form formFromString(std::string_view str) {
	if (str == "solid") return Item::Form::solid;
	if (str == "liquid") return Item::Form::liquid;
	if (str == "gas") return Item::Form::gas;
	return Item::Form::solid;
}

Item itemFromJson(const json& j) {
	return Item{ .id{j["id"]}, .displayName{j["displayName"]}, .form{formFromString(j["form"])} };
}