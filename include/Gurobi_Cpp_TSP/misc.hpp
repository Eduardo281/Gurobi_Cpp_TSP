#pragma once

#include <string>
#include <vector>

// NOTE: Right now, both must have the same type. Future developments might allow different types!
using COORDINATES_TYPE = double;
using DISTANCE_TYPE    = double;

enum class ATSP_ModelType { DFJ, MTZ, GG };
enum class ObjectiveType { MIN_DIST, MAX_DIST, MINMAX_EDGE, MAXMIN_EDGE };

GRBEnv* buildGurobiEnv() { return new GRBEnv(); };
std::string str(size_t i) { return std::to_string(i); };

class Point {
public:
    Point() {
        this->points = std::vector<COORDINATES_TYPE>();
        this->dimensions = 0;
    };

    Point(std::vector<COORDINATES_TYPE> inputVector){
        this->points.reserve(inputVector.size());
        for (size_t i{0}; i < inputVector.size(); ++i)
            this->points.push_back(inputVector.at(i));
        this->dimensions = this->points.size();
    };

    const std::vector<COORDINATES_TYPE>& getCoordinates() const { return points; };
    const COORDINATES_TYPE getCoordinateAt(size_t pos) const { return points.at(pos); };
    const size_t getDimensions() const { return dimensions; };
private:
    std::vector<COORDINATES_TYPE> points;
    size_t dimensions;
};
