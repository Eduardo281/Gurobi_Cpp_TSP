#include "gurobi_c++.h"

#include "include/Gurobi_Cpp_TSP.hpp"

#include "third_party/rapidcsv.h"

int main(){
    const std::string INSTANCE_NAME{ "burma14.csv" };
    const std::string INSTANCE_PATH{ "./Instances/" + INSTANCE_NAME };

    std::vector<Point> points{ readInstanceFile(INSTANCE_PATH) };

    GRBEnv* env{ buildGurobiEnv() };

    TSPInput data{ TSPInput(points, ObjectiveType::MIN_DIST, distanceFunction_Euclidean) };

    TSPCreator factory{ TSPCreator(env, data) };

    ATSPModel* dfjModel{ factory.build(ATSP_ModelType::DFJ) };
    dfjModel->solve();
    dfjModel->printSolution();

    ATSPModel* mtzModel{ factory.build(ATSP_ModelType::MTZ) };
    mtzModel->solve();
    mtzModel->printSolution();

    ATSPModel* ggModel{ factory.build(ATSP_ModelType::GG) };
    ggModel->solve();
    ggModel->printSolution();
}
