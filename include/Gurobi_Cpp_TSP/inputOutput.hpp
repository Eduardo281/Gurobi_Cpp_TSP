#pragma once

#include <vector>

#include "misc.hpp"
#include "distanceFunctions.hpp"

class TSPInput {
public:
    TSPInput() {};
    TSPInput(
        std::vector<Point> inputPoints,
        ObjectiveType objType = ObjectiveType::MIN_DIST,
        DISTANCE_TYPE(*inputObjFunction)(Point, Point) = distanceFunction_Euclidean
    ) {
        this->points = inputPoints;
        this->qtyOfPoints = this->points.size();
        this->objectiveType = objType;
        this->objectiveFunction = inputObjFunction;
    };
    const size_t getQtyOfPoints() const { return this->qtyOfPoints; };
    const ObjectiveType getObjectiveType() const { return this->objectiveType; };
    const DISTANCE_TYPE getDistance(size_t i, size_t j) {
        return this->objectiveFunction(this->points.at(i), this->points.at(j));
    };
    void setObjectiveType(ObjectiveType objType) { this->objectiveType = objType; };
private:
    std::vector<Point> points{};
    size_t qtyOfPoints{};
    ObjectiveType objectiveType{ ObjectiveType::MIN_DIST };
    DISTANCE_TYPE(*objectiveFunction)(Point, Point) { distanceFunction_Euclidean };
};

struct TSPSolution {
    std::vector<size_t> route{};
    double value{};
    double runtime{};
    double MIPGap{};
    int gurobiSolveCode{};
    bool foundSolution{};
};
