#pragma once

#include "Matrix.h"
#include <optional>
#include <utility>
#include <variant>
#include <vector>


enum class SimplexError {
	InvalidInput,
	Infeasible
};

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
/// <param name="A">A: constraint coefficients</param>
/// <param name="b">b: constraint bounds</param>
/// <param name="c">c: goal function</param>
/// <param name="minimize">whether to minimize the goal function instead (default: false, i.e. maximize)</param>
/// <returns>x: solution, or empty if infeasible or unbounded</returns>
std::variant<std::vector<double>, SimplexError> simplex(
	const std::vector<std::vector<double>>& A,
	const std::vector<double>& b,
	const std::vector<double>& c,
	bool minimize = false
);

namespace {
	struct SimplexMaximizeResult {
		std::vector<double> x;
		std::vector<double> s;
	};

	/// <summary>
	/// Main algorithm for the simplex goal-maximization problem.
	/// Same arguments as for `simplex`.
	/// </summary>
	/// <returns>An object containing the solution x and final slack values s, or empty if infeasible or unbounded.</returns>
	std::optional<SimplexMaximizeResult> simplexMaximize(
		const Matrix<double>& A,
		const std::vector<double>& b,
		const std::vector<double>& c
	);

	/// <summary>
	/// A tableu represents an optimal solution when all values in the top row are >= 0.
	/// </summary>
	/// <param name="tableu">Reference to the tableu to check</param>
	/// <returns>whether the tableu represents an optimal solution</returns>
	bool isOptimal(
		const std::vector<std::vector<double>>& tableu
	);

	/// <summary>
	/// The next pivot to pick is any column n where the top row has a negative value, and then pick the row to annhilate
	/// that value by choosing m such that b[m] / a[m][n] is minimized (but still >=0).
	/// 
	/// Must only be called if tableu is not optimal, otherwise an exception is thrown.
	/// </summary>
	/// <param name="tableu">Reference to the tableu to check</param>
	/// <returns>The row and column to pivot with (or empty if infeasible)</returns>
	std::optional<std::pair<std::size_t, std::size_t>> pickPivot(
		const std::vector<std::vector<double>>& tableu
	);

	/// <summary>
	/// Perform a pivot operation. This consists of zeroing out all the values in column n except row m, by subtracting
	/// multiples of row m.
	/// 
	/// Mutates the tableu in place.
	/// </summary>
	/// <param name="tableu">Reference to the tableu to check</param>
	/// <param name="mPivot">The row to pivot with</param>
	/// <param name="nPivot">The column to zero out</param>
	void doPivot(
		std::vector<std::vector<double>>& tableu,
		std::size_t mPivot,
		std::size_t nPivot
	);
}