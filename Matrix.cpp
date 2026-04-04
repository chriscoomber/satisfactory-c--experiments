#include "Matrix.h"
#include <optional>
#include <vector>

template <typename T>
Matrix<T>::Matrix(const std::vector<std::vector<T>> data, std::size_t height, std::size_t width)
	: mData{ data }, mHeight{ height }, mWidth{ width } {
}

template <typename T>
const std::vector<std::vector<T>>& Matrix<T>::data() const { return mData; }

template <typename T>
const std::size_t Matrix<T>::height() const { return mHeight; }

template <typename T>
const std::size_t Matrix<T>::width() const { return mWidth; }

template <typename T>
std::optional<Matrix<T>> Matrix<T>::create(const std::vector<std::vector<T>> data) {
	// Validation
	const std::size_t M{ data.size() };
	const std::size_t N{ data[0].size() };

	// Check that every other row is N wide.
	for (std::size_t m{ 1 }; m < M; ++m) {
		if (data[m].size() != N) return {};
	}

	return Matrix{ data, M, N };
}

template <typename T>
Matrix<T> Matrix<T>::transpose() const
{
	std::vector<std::vector<T>> newData(mWidth, std::vector<T>(mHeight));
	for (std::size_t m{ 0 }; m < mHeight; ++m) {
		for (std::size_t n{ 0 }; n < mWidth; ++n) {
			newData[n][m] = mData[m][n];
		}
	}

	return Matrix{ newData, mWidth, mHeight };
}

template <typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& other) const
{
	std::vector<std::vector<T>> newData(mHeight, std::vector<T>(other.mWidth));

	for (std::size_t m{ 0 }; m < mHeight; ++m) {
		for (std::size_t n{ 0 }; n < other.mWidth; ++n) {
			for (std::size_t k{ 0 }; k < mWidth; ++k)
				newData[m][n] += mData[m][k] * other.mData[k][n];
		}
	}
	return Matrix{ newData, mHeight, other.mWidth };
}

// Because this is a stupid language, I either have to define these functions in the header (bad), or only create
// actual implementations for specific values of T (also bad, but that's what we do here)
template class Matrix<double>;
template class Matrix<int>;
