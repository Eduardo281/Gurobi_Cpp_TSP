#pragma once

#include <cmath>
#include <iostream>
#include <limits>

#include "misc.hpp"
#include "exceptions.hpp"

DISTANCE_TYPE distanceFunction_Euclidean(Point p1, Point p2) {
	DISTANCE_TYPE result{ 0 };
	if (p1.getDimensions() != p2.getDimensions()) {
		throw TSPDistanceFunctionBadInput();
	}
	for (size_t i{ 0 }; i < p1.getDimensions(); ++i) {
		result += std::pow(p1.getCoordinateAt(i) - p2.getCoordinateAt(i), 2);
	}
	if (std::numeric_limits<DISTANCE_TYPE>::is_integer)
		return static_cast<DISTANCE_TYPE>(std::sqrt(1.0 * result) + 0.5);
	else
	return std::sqrt(result);
}

DISTANCE_TYPE distanceFunction_Manhattan(Point p1, Point p2) {
	DISTANCE_TYPE result{ 0 };
	if (p1.getDimensions() != p2.getDimensions()) {
		throw TSPDistanceFunctionBadInput();
	}
	for (size_t i{ 0 }; i < p1.getDimensions(); ++i) {
		result += std::abs(p1.getCoordinateAt(i) - p2.getCoordinateAt(i));
	}
	return result;
}

DISTANCE_TYPE distanceFunction_Max(Point p1, Point p2) {
	DISTANCE_TYPE result{ 0 };
	if (p1.getDimensions() != p2.getDimensions()) {
		throw TSPDistanceFunctionBadInput();
	}
	for (size_t i{ 0 }; i < p1.getDimensions(); ++i) {
		result = std::max(result, std::abs(p1.getCoordinateAt(i) - p2.getCoordinateAt(i)));
	}
	return result;
}
