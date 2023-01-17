#pragma once

#include <limits>

// math functions from the standard library rewritten to be constexpr
double constexpr sqrtNewtonRaphson(double x, double curr, double prev) {
    return curr == prev
        ? curr
        : sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}

/*
 * Constexpr version of the square root
 * Return value:
 *	- For a finite and non-negative value of "x", returns an approximation for the square root of "x"
 *   - Otherwise, returns NaN
 */
// code from: https://gist.github.com/alexshtf/eb5128b3e3e143187794
constexpr double constSqrt(double x) {
    return x >= 0 && x < std::numeric_limits<double>::infinity()
        ? sqrtNewtonRaphson(x, x, 0)
        : std::numeric_limits<double>::quiet_NaN();
}