#pragma once

#include <string>
#include <exception>

class TSPDistanceFunctionBadInput : public std::exception {
public:
	const char* what() const noexcept override {
		return "Exception occurried while reading input on distance function!\nPoints passed as input must have equal number of dimensions!";
	}
};

class TSPFactoryBadInput : public std::exception {
public:
	const char* what() const noexcept override {
		return "Exception occurried on TSP Factory!\nBad input received!";
	}
};

class TSPObjectiveFunctionBadInput : public std::exception {
public:
	const char* what() const noexcept override {
		return "Exception occurried while building a model objective function!\nBad input received!";
	}
};

class TSPModelBadInput : public std::exception {
public:
	const char* what() const noexcept override {
		return "Exception occurried while building the model.\nModel input must have at least 3 points.\nBad input received!";
	}
};