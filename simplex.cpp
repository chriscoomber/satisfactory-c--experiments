#include "Matrix.h"
#include "simplex.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

std::variant<std::vector<double>, SimplexError> simplex(
	const std::vector<std::vector<double>>& A,
	const std::vector<double>& b,
	const std::vector<double>& c,
	bool minimize
)
{
	const std::optional<Matrix<double>> AMatrix = Matrix<double>::create(A);
	if (!AMatrix) return SimplexError::InvalidInput;

	// Validate that b is height M and c is width N
	if (b.size() != AMatrix->height()) return SimplexError::InvalidInput;
	if (c.size() != AMatrix->width()) return SimplexError::InvalidInput;

	if (minimize) {
		// First call simplexMaximize, but with some adjusted parameters:
		// A -> A transpose
		// b -> c
		// c -> b
		// This is the "dual LP"
		auto MTranspose = AMatrix->transpose();
		auto result{ simplexMaximize(MTranspose, c, b) };
		if (!result) return SimplexError::Infeasible;

		// Then the solution to our original ("primal") LP are the slack parameters
		return result->s;
	}
	else {
		// Just call simplexMaximize
		auto result{ simplexMaximize(*AMatrix, b, c) };
		if (!result) return SimplexError::Infeasible;
		return result->x;
	}
}

namespace {

	std::optional<SimplexMaximizeResult> simplexMaximize(
		const Matrix<double>& A,
		const std::vector<double>& b,
		const std::vector<double>& c
	)
	{
		const std::size_t M{ A.height() };
		const std::size_t N{ A.width() };

		// Imagine a block matrix (aka tableu) of the following form: (where m=2, n=3 as an example)
		//
		// 1 -c[1]   -c[2]   -c[3]   0 0 | 0
		// 0 a[1][1] a[1][2] a[1][3] 1 0 | b[1]
		// 0 a[2][1] a[2][2] a[2][3] 0 1 | b[2]
		//
		// And imagine multiplying LHS with the column vector (z, x[1], x[2], x[3], s[1], s[2]) and setting = RHS.
		// 
		// This represents the following equations:
		// 
		// z - c[1]*x[1] - c[2]*x[2] -c[3]*x[3] = 0, where z is the value of the goal achieved
		// a[m][1]*x[1] + a[m][2]*x[2] + a[m][3]*x[3] + s[m] = b[m], for all m=1,...,M, where s[m] is a slack variable
		// 
		// Or in other words:
		// 
		// c.x = z
		// Ax + s = b
		//
		// Our task is to apply a series of "pivot" operations to make the coefficients of the top row positive. This
		// implies our solution is optimal. Then, after zeroeing out any variables with non-zero coefficients in the top
		// row, we should have some very simple equations for x[1],...,x[n] which we can read off. This is just a quick
		// summary, so if anything doesn't make sense, consider looking up the theory.
		std::vector<std::vector<double>> tableu(1 + M, std::vector<double>(1 + N + M + 1, 0.0));

		// Copy values across to the tableu
		tableu[0][0] = 1.0;
		for (std::size_t n{ 0 }; n < N; ++n) {
			tableu[0][1 + n] = -c[n];
		}
		for (std::size_t m{ 0 }; m < M; ++m) {
			for (std::size_t n{ 0 }; n < N; ++n) {
				tableu[1 + m][1 + n] = A.data()[m][n];
			}
		}
		for (std::size_t m{ 0 }; m < M; ++m) {
			tableu[1 + m][1 + N + m] = 1;
		}
		for (std::size_t m{ 0 }; m < M; ++m) {
			tableu[1 + m][1 + N + M] = b[m];
		}

		// Perform the algorithm
		while (!isOptimal(tableu)) {
			auto pivots{ pickPivot(tableu) };
			// Can we continue? Or is it unfeasible/unbounded?
			if (!pivots) return {};
			auto mPivot{ pivots->first };
			auto nPivot{ pivots->second };

			doPivot(tableu, mPivot, nPivot);
		}


		// Read off the solution
		std::vector<double> x(N, 0.0);
		for (std::size_t n{ 0 }; n < N; ++n) {
			// Read off the value for the row in which this column is non-zero
			for (std::size_t m{ 0 }; m < M; ++m) {
				if (std::abs(tableu[m + 1][n + 1]) > 1e-5) {
					// This is the row which defines x[n]
					x[n] = tableu[m + 1][1 + N + M] / tableu[m + 1][n + 1];
				}

			}
		}

		// Also read off the final slack values, as sometimes these are useful
		std::vector<double> s(M, 0.0);
		for (std::size_t m{ 0 }; m < M; ++m) {
			s[m] = tableu[0][1 + N + m];
		}

		return SimplexMaximizeResult{ .x{ x }, .s{ s } };
	}

	bool isOptimal(
		const std::vector<std::vector<double>>& tableu
	)
	{
		const std::vector<double>& firstRow = tableu[0];
		const double minElement{ *std::min_element(firstRow.begin(), firstRow.end()) };

		std::cout << "Is optimal? Min element: " << minElement << "\n";

		return minElement >= 0.0;
	}

	std::optional<std::pair<std::size_t, std::size_t>> pickPivot(
		const std::vector<std::vector<double>>& tableu
	)
	{
		std::size_t nPivot = 0; // 0 is a sentinel value that indicates not found, since row 0 is never a valid column pivot
		double bestPivot = 0.0;
		for (std::size_t n{ 1 }; n < tableu[0].size(); ++n) {
			if (tableu[0][n] < bestPivot) {
				bestPivot = tableu[0][n];
				nPivot = n;
				break;
			}
		}
		if (!nPivot) {
			throw std::runtime_error("Must be an nPivot if not optimal...");
		}

		std::size_t mPivot = 0; // 0 is a sentinel value that indicates not found, since row 0 is never a valid row pivot
		//double bestRatio = DBL_MAX;
		for (std::size_t m{ 1 }; m < tableu.size(); ++m) {
			double denom = tableu[m][nPivot];
			if (denom == 0.0) continue;
			double ratio = tableu[m].back() / denom;
			if (ratio >= 0.0 /* && ratio < bestRatio*/) {
				//bestRatio = ratio;
				mPivot = m;
				break;
			}
		}
		if (!mPivot) {
			// The solution is infeasible or unbounded
			// TODO: how do you tell the difference?
			return {};
		}

		return std::pair(mPivot, nPivot);
	}

	void doPivot(
		std::vector<std::vector<double>>& tableu,
		std::size_t mPivot,
		std::size_t nPivot
	)
	{
		// For each row other than row m, add some multiple of row m, to zero out the nth column
		for (std::size_t m{ 0 }; m < tableu.size(); ++m) {
			if (m == mPivot) continue;

			double coefficient = tableu[m][nPivot] / tableu[mPivot][nPivot];

			for (std::size_t n{ 0 }; n < tableu[m].size(); ++n) {
				tableu[m][n] -= coefficient * tableu[mPivot][n];
			}
		}
	}
}