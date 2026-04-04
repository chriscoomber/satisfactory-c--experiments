#pragma once

#include <optional>
#include <vector>

template <typename T>
class Matrix {
private:
	const std::vector<std::vector<T>> mData;
	const std::size_t mHeight;
	const std::size_t mWidth;

	Matrix(const std::vector<std::vector<T>> data, std::size_t height, std::size_t width);

public:
	const std::vector<std::vector<T>>& data() const;
	const std::size_t height() const;
	const std::size_t width() const;

	static std::optional<Matrix> create(const std::vector<std::vector<T>> data);

	Matrix transpose() const;

	Matrix operator*(const Matrix& other) const;
};
