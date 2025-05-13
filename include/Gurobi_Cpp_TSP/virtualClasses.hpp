#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "gurobi_c++.h"

#include "misc.hpp"
#include "inputOutput.hpp"

class TSPBaseModel{
public:
	TSPBaseModel(GRBEnv* envPtr, TSPInput& inputData) {
		model = new GRBModel(envPtr);
		data = inputData;

		if (data.getQtyOfPoints() < 3) {
			throw TSPModelBadInput();
		}

		vertices.reserve(data.getQtyOfPoints());
		for (size_t i{ 1 }; i <= data.getQtyOfPoints(); ++i) {
			vertices.push_back(i);
		}
	};
	TSPSolution& getSolution() { return this->solution; };
	void solve();
	void printSolution();

protected:
	GRBModel* model;

	TSPInput data;
	TSPSolution solution;
	std::unordered_map<std::string, GRBVar> vars;
	std::string modelAlias;

	std::vector<size_t> vertices;

	const std::string getModelAlias() const { return this->modelAlias; };

	const std::string varIdx(std::string varName, std::vector<size_t> indexes);
	const std::string varIdx(std::string varName, size_t index);
	const std::string cstrIdx(std::string cstrName, std::vector<size_t> indexes);
	const std::string cstrIdx(std::string cstrName, size_t index);

	virtual void buildModel() = 0;

	DISTANCE_TYPE getDistance(size_t i, size_t j) { return data.getDistance(i - 1, j - 1); };

	void updateSolution();
	void createVars_X();
	void createVars_U();
	void createVars_G();
	void createObjectiveFunction();
	void createCstr_EveryCityIsLeavedOnce();
	void createCstr_EveryCityIsVisitedOnce();
	void createCstr_DFJSubtourRemoval();
	void createCstr_MTZSubtourRemoval();
	void createCstr_GVarsFlow();
	void createConstr_GVarsBoundedByX();
};

const std::string TSPBaseModel::varIdx(std::string varName, std::vector<size_t> indexes) {
	std::string res{ varName };
	for (size_t pos{ 0 }; pos < indexes.size(); ++pos) {
		res += "_" + str(indexes.at(pos));
	}
	return res;
}

const std::string TSPBaseModel::varIdx(std::string varName, size_t index) {
	return this->varIdx(varName, std::vector<size_t>({ index }));
};

const std::string TSPBaseModel::cstrIdx(std::string cstrName, std::vector<size_t> indexes) {
	std::string res{ cstrName };
	for (size_t pos{ 0 }; pos < indexes.size(); ++pos) {
		res += "_" + str(indexes.at(pos));
	}
	return res;
};

const std::string TSPBaseModel::cstrIdx(std::string cstrName, size_t index) {
	return this->cstrIdx(cstrName, std::vector<size_t>({ index }));
};

void TSPBaseModel::updateSolution() {
	try { 
		this->solution.value = this->model->get(GRB_DoubleAttr_ObjVal); 
		this->solution.MIPGap = this->model->get(GRB_DoubleAttr_MIPGap);
		this->solution.foundSolution = true;
	}
	catch (const std::exception& e) { 
		this->solution.value = 0; 
		this->solution.MIPGap = 0;
		this->solution.foundSolution = false;
	}
	
	try {
		this->solution.runtime = this->model->get(GRB_DoubleAttr_Runtime);
	}
	catch (const std::exception& e) {
		this->solution.runtime = 0;
	}
	
	this->solution.gurobiSolveCode = this->model->get(GRB_IntAttr_Status);

	if (this->solution.foundSolution) {
		this->solution.route.clear();
		this->solution.route.reserve(this->data.getQtyOfPoints());
		size_t p0 = 1;
		this->solution.route.push_back(p0);
		for (size_t k{ 1 }; k < this->data.getQtyOfPoints(); ++k) {
			for (size_t p1 : vertices) {
				if (p0 != p1) {
					if(vars.at(varIdx("x", {p0, p1})).get(GRB_DoubleAttr_X) > 0.5) {
						p0 = p1;
						this->solution.route.push_back(p0);
						break;
					}
				}
			}
		}
	}
	else {
		this->solution.route = std::vector<size_t>{};
	}
}

void TSPBaseModel::solve() {
	this->model->optimize();
	this->updateSolution();
}

void TSPBaseModel::printSolution() {
	std::cout << "===================" << std::endl;
	std::cout << "Solution found for model " << this->getModelAlias() << std::endl;
	std::cout << "Route built: " << this->solution.route.at(0);
	for (size_t i{ 1 }; i < this->solution.route.size(); ++i) {
		std::cout << " -> " << this->solution.route.at(i);
	}
	std::cout << std::endl;
	std::cout << "Solution value: " << this->solution.value << std::endl;
	std::cout << "Runtime: " << this->solution.runtime << "s" << std::endl;
	std::cout << "GAP: " << this->solution.MIPGap << std::endl;
	std::cout << "===================" << std::endl;
};

void TSPBaseModel::createVars_X(){
	for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
		for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				vars.insert({
					varIdx("x", {*it_i, *it_j}),
					this->model->addVar(0, 1, 0, GRB_BINARY, NULL)
				});
			}
		}
	}
}

void TSPBaseModel::createVars_U() {
	for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
		vars.insert({
			varIdx("u", *it_i),
			this->model->addVar(0, static_cast<double>(this->data.getQtyOfPoints())-1, 0, GRB_CONTINUOUS, NULL)
		});
	}
}

void TSPBaseModel::createVars_G() {
	for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
		for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				vars.insert({
					varIdx("g", {*it_i, *it_j}),
					this->model->addVar(0, this->data.getQtyOfPoints() - 1, 0, GRB_CONTINUOUS, NULL)
				});
			}
		}
	}
}

void TSPBaseModel::createObjectiveFunction() {
	GRBVar objVar{ this->model->addVar(-GRB_INFINITY, GRB_INFINITY, NULL, GRB_CONTINUOUS, "objVar") };
	GRBLinExpr objExpr{ NULL };
	switch (this->data.getObjectiveType()) {
	case ObjectiveType::MIN_DIST:
	case ObjectiveType::MAX_DIST:
		for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
			for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
				if (*it_i != *it_j) {
					objExpr += this->getDistance(*it_i, *it_j) * vars.at(varIdx("x", {*it_i, *it_j}));
				}
			}
		}
		break;
	case ObjectiveType::MINMAX_EDGE:
		for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
			for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
				if (*it_i != *it_j) {
					this->model->addConstr(
						objVar >= this->getDistance(*it_i, *it_j) * vars.at(varIdx("x", {*it_i, *it_j})),
						cstrIdx("minmax_edge", {*it_i, *it_j})
					);
				}
			}
		}
		break;
	case ObjectiveType::MAXMIN_EDGE:
		for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
			objExpr = NULL;
			for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
				if (*it_i != *it_j) {
					objExpr += this->getDistance(*it_i, *it_j) * vars.at(varIdx("x", {*it_i, *it_j}));
				}
			}
			this->model->addConstr(objVar <= objExpr, cstrIdx("maxmin_edge", *it_i));
		}
		break;
	default:
		throw TSPObjectiveFunctionBadInput();
		break;
	}

	switch (this->data.getObjectiveType()) {
	case ObjectiveType::MIN_DIST:
		this->model->setObjective(objExpr, GRB_MINIMIZE);
		break;
	case ObjectiveType::MINMAX_EDGE:
		this->model->setObjective(static_cast<GRBLinExpr>(objVar), GRB_MINIMIZE);
		break;
	case ObjectiveType::MAX_DIST:
		this->model->setObjective(objExpr, GRB_MAXIMIZE);
		break;
	case ObjectiveType::MAXMIN_EDGE:
		this->model->setObjective(static_cast<GRBLinExpr>(objVar), GRB_MAXIMIZE);
		break;
	default:
		break;
	}
}

void TSPBaseModel::createCstr_EveryCityIsLeavedOnce() {
	for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
		GRBLinExpr expr{ NULL };
		for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				expr += vars.at(varIdx("x", { *it_i, *it_j }));
			}
		}
		this->model->addConstr(expr == 1, cstrIdx("cstr1", *it_i));
	}
}

void TSPBaseModel::createCstr_EveryCityIsVisitedOnce() {
	for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
		GRBLinExpr expr{ NULL };
		for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
			if (*it_i != *it_j) {
				expr += vars.at(varIdx("x", { *it_i, *it_j }));
			}
		}
		this->model->addConstr(expr == 1, cstrIdx("cstr2", *it_j));
	}
}

void TSPBaseModel::createCstr_DFJSubtourRemoval() {
	unsigned char n{ static_cast<unsigned char>(this->data.getQtyOfPoints()) };
	
	size_t nextSkipCase{ 1 };
	unsigned char nextSkipCasePow{ 0 };

	for (size_t i{ 1 }; i < (static_cast<size_t>(1 << n) - 1); i++) {
		if (i == nextSkipCase) {
			nextSkipCasePow += 1;
			nextSkipCase = static_cast<size_t>(1 << nextSkipCasePow);
			continue;
		}

		std::vector<unsigned char> sub{};

		for (unsigned char j{ 0 }; j < n; j++)
			if (i & static_cast<size_t>(1 << j))
				sub.push_back(vertices.at(j));

		GRBLinExpr expr{ NULL };
		for (unsigned char i : sub) {
			for (unsigned char j : sub) {
				if (i != j) {
					expr += vars.at(varIdx("x", { i, j }));
				}
			}
		}
		this->model->addConstr(expr <= sub.size() - 1, cstrIdx("cstrDFJ", i));
	}
}

void TSPBaseModel::createCstr_MTZSubtourRemoval() {
	for (auto it_i{ vertices.begin() + 1 }; it_i != vertices.end(); ++it_i) {
		for (auto it_j{ vertices.begin() + 1 }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				GRBLinExpr expr{ vars.at(varIdx("u", *it_i)) - vars.at(varIdx("u", *it_j)) + 1 };
				this->model->addConstr(
					expr <= (this->data.getQtyOfPoints()-1) * ( 1 - vars.at(varIdx("x", {*it_i, *it_j})) ),
					cstrIdx("cstrMTZ", { *it_i, *it_j })
				);
			}
		}
	}
};

void TSPBaseModel::createCstr_GVarsFlow() {
	for (auto it_i{ vertices.begin() + 1}; it_i != vertices.end(); ++it_i) {
		GRBLinExpr expr{ NULL };
		for (auto it_j{ vertices.begin() }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				expr += vars.at(varIdx("g", { *it_j, *it_i }));
			}
		}
		for (auto it_j{ vertices.begin() + 1 }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				expr -= vars.at(varIdx("g", { *it_i, *it_j }));
			}
		}
		this->model->addConstr(expr == 1, cstrIdx("GFlow", *it_i));
	}
};

void TSPBaseModel::createConstr_GVarsBoundedByX() {
	for (auto it_i{ vertices.begin() }; it_i != vertices.end(); ++it_i) {
		for (auto it_j{ vertices.begin() + 1 }; it_j != vertices.end(); ++it_j) {
			if (*it_i != *it_j) {
				this->model->addConstr(
					vars.at(varIdx("g", { *it_i, *it_j })) <= (data.getQtyOfPoints()-1) * vars.at(varIdx("x", { *it_i, *it_j })),
					cstrIdx("GLEX", {*it_i, *it_j})
				);
			}
		}
	}
};

class TSPAssignmentBaseProblem : public TSPBaseModel {
public:
	TSPAssignmentBaseProblem(GRBEnv* env, TSPInput& inputData) : TSPBaseModel(env, inputData) {
		this->createVars_X();
		this->createObjectiveFunction();
		this->createCstr_EveryCityIsVisitedOnce();
		this->createCstr_EveryCityIsLeavedOnce();
	};
};

class ATSPModel : public TSPAssignmentBaseProblem {
public:
	ATSPModel(GRBEnv* env, TSPInput& inputData) : TSPAssignmentBaseProblem(env, inputData) {};
};
