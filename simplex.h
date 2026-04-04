#pragma once

#include <algorithm>
#include <array>
#include <cfloat>
#include <optional>
#include <stdexcept>
#include <utility>
#include <valarray>

/// <summary>
/// Solve the linear programming problem: Ax <= b, x>=0, maximizing c.x, where:
/// 
/// - M is the number of constraints
/// - N is the number of variables
/// - x is the solution vector (x[1],...,x[N])
/// - A (MxN matrix) and b (M vector) form the constraints, i.e. a[m,1] * x[1] + ... + a[m,N] * x[N] <= b[m], for all m=1,...,M
/// - c (N vector) is the goal vector, so we're maximizing c[1]x[1] + ... + c[N]x[N]
/// - x[n] >= 0 for all n=1,...,N
/// 
/// If minimize is set, then this function instead solves Ax >= b, x>=0, minimizing c.x.
/// </summary>
/// <typeparam name="M">the number of constraints</typeparam>
/// <typeparam name="N">the number of variables</typeparam>
/// <param name="A">A: constraint coefficients</param>
/// <param name="b">b: constraint bounds</param>
/// <param name="c">c: goal function</param>
/// <param name="minimize">whether to minimize the goal function instead (default: false, i.e. maximize)</param>
/// <returns>x: solution, or empty if infeasible or unbounded</returns>
template <std::size_t M, std::size_t N>
std::optional<std::array<double, N>> simplex(
	const std::array<std::array<double, N>, M>& A,
	const std::array<double, M>& b,
	const std::array<double, N>& c,
	bool minimize = false
)
{
	if (minimize) {
		// First call simplexMaximize, but with some adjusted parameters:
		// A -> A transpose
		// b -> c
		// c -> b
		// This is the "dual LP"
		auto result{ simplexMaximize<N,M>(transpose<M,N>(A), c, b) };
		if (!result) return {};

		// Then the solution to our original ("primal") LP are the slack parameters
		return result->s;
	}
	else {
		// Just call simplexMaximize
		auto result{ simplexMaximize<M,N>(A, b, c) };
		if (!result) return {};
		return result->x;
	}
}

/// <summary>
/// Transpose an MxN matrix. Creates a new matrix, doesn't mutate the original.
/// </summary>
/// <param name="A">The matrix to transpose</param>
/// <returns>The NxM matrix A transposed</returns>
template <std::size_t M, std::size_t N>
const std::array<std::array<double, M>, N> transpose(
	const std::array<std::array<double, N>, M>& A
)
{
	std::array<std::array<double, M>, N> ATranspose{};
	for (std::size_t m{ 0 }; m < M; ++m) {
		for (std::size_t n{ 0 }; n < N; ++n) {
			ATranspose[n][m] = A[m][n];
		}
	}

	return ATranspose;
}

template <std::size_t M, std::size_t N>
struct SimplexMaximizeResult {
	std::array<double, N> x;
	std::array<double, M> s;
};

/// <summary>
/// Main algorithm for the simplex goal-maximization problem.
/// Same arguments as for `simplex`.
/// </summary>
/// <returns>An object containing the solution x and final slack values s, or empty if infeasible or unbounded.</returns>
template <std::size_t M, std::size_t N>
std::optional<SimplexMaximizeResult<M, N>> simplexMaximize(
	const std::array<std::array<double, N>, M>& A,
	const std::array<double, M>& b,
	const std::array<double, N>& c
)
{
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
	std::array<std::array<double, 1 + N + M + 1>, 1 + M> tableu{};

	// Copy values across to the tableu
	tableu[0][0] = 1.0;
	for (std::size_t n{ 0 }; n < N; ++n) {
		tableu[0][1 + n] = -c[n];
	}
	for (std::size_t m{ 0 }; m < M; ++m) {
		for (std::size_t n{ 0 }; n < N; ++n) {
			tableu[1 + m][1 + n] = A[m][n];
		}
	}
	for (std::size_t m{ 0 }; m < M; ++m) {
		tableu[1 + m][1 + N + m] = 1;
	}
	for (std::size_t m{ 0 }; m < M; ++m) {
		tableu[1 + m][1 + N + M] = b[m];
	}

	// Perform the algorithm
	while (!isOptimal<M, N>(tableu)) {
		auto pivots{ pickPivot<M, N>(tableu) };
		// Can we continue? Or is it unfeasible/unbounded?
		if (!pivots) return {};
		auto mPivot{ pivots->first };
		auto nPivot{ pivots->second };

		doPivot<M, N>(tableu, mPivot, nPivot);
	}


	// Read off the solution
	std::array<double, N> x{};
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
	std::array<double, M> s{};
	for (std::size_t m{ 0 }; m < M; ++m) {
		s[m] = tableu[0][1 + N + m];
	}

	return SimplexMaximizeResult<M, N>{.x{ x }, .s{ s } };
}

/// <summary>
/// A tableu represents an optimal solution when all values in the top row are >= 0.
/// </summary>
/// <typeparam name="M">the number of constraints</typeparam>
/// <typeparam name="N">the number of variables</typeparam>
/// <param name="tableu">Reference to the tableu to check</param>
/// <returns>whether the tableu represents an optimal solution</returns>
template <std::size_t M, std::size_t N>
bool isOptimal(
	const std::array<std::array<double, 1 + N + M + 1>, 1 + M>& tableu
)
{
	const std::array<double, 1 + N + M + 1>& firstRow = tableu[0];

	return *std::min_element(firstRow.begin(), firstRow.end()) >= 0.0;
}

/// <summary>
/// The next pivot to pick is any column n where the top row has a negative value, and then pick the row to annhilate
/// that value by choosing m such that b[m] / a[m][n] is minimized (but still >=0).
/// </summary>
/// <typeparam name="M">the number of constraints</typeparam>
/// <typeparam name="N">the number of variables</typeparam>
/// <param name="tableu">Reference to the tableu to check</param>
/// <returns>The row and column to pivot with (or empty if infeasible)</returns>
template <std::size_t M, std::size_t N>
std::optional<std::pair<std::size_t, std::size_t>> pickPivot(
	const std::array<std::array<double, 1 + N + M + 1>, 1 + M>& tableu
)
{
	std::size_t nPivot = 0; // 0 is a sentinel value that indicates not found, since row 0 is never a valid column pivot
	double bestPivot = 0.0;
	for (std::size_t n{ 1 }; n < 1 + N + M + 1; ++n) {
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
	double bestRatio = DBL_MAX;
	for (std::size_t m{ 1 }; m < 1 + M; ++m) {
		double ratio = tableu[m][1 + N + M] / tableu[m][nPivot];
		if (ratio >= 0.0 && ratio < bestRatio) {
			bestRatio = ratio;
			mPivot = m;
		}
	}
	if (!mPivot) {
		// The solution is infeasible or unbounded
		// TODO: how do you tell the difference?
		return {};
	}

	return std::pair(mPivot, nPivot);
}

/// <summary>
/// Perform a pivot operation. This consists of zeroing out all the values in column n except row m, by subtracting
/// multiples of row m.
/// 
/// Mutates the tableu in place.
/// </summary>
/// <typeparam name="M">the number of constraints</typeparam>
/// <typeparam name="N">the number of variables</typeparam>
/// <param name="tableu">Reference to the tableu to check</param>
/// <param name="mPivot">The row to pivot with</param>
/// <param name="nPivot">The column to zero out</param>
template <std::size_t M, std::size_t N>
void doPivot(
	std::array<std::array<double, 1 + N + M + 1>, 1 + M>& tableu,
	std::size_t mPivot,
	std::size_t nPivot
)
{
	// For each row other than row m, add some multiple of row m, to zero out the nth column
	for (std::size_t m{ 0 }; m < 1 + M; ++m) {
		if (m == mPivot) continue;

		double coefficient = tableu[m][nPivot] / tableu[mPivot][nPivot];

		for (std::size_t n{ 0 }; n < 1 + N + M + 1; ++n) {
			tableu[m][n] -= coefficient * tableu[mPivot][n];
		}
	}
}
