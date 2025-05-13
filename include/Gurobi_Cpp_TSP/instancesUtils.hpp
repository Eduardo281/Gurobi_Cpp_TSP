#pragma once

#include <string>
#include <vector>

#include "misc.hpp"
#include "../../third_party/rapidcsv.h"

std::vector<Point> readInstanceFile(std::string filePath){
    rapidcsv::Document doc(filePath, rapidcsv::LabelParams(-1, -1));

    std::vector<COORDINATES_TYPE> x{doc.GetColumn<COORDINATES_TYPE>(1)};
    std::vector<COORDINATES_TYPE> y{doc.GetColumn<COORDINATES_TYPE>(2)};

    std::vector<Point> data{};
    data.reserve(x.size());

    for(size_t i{0}; i < x.size(); ++i){
        data.push_back(Point({x.at(i), y.at(i)}));
    }

    return data;
}
