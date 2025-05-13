#pragma once

#include "virtualClasses.hpp"

class ATSPModel_DFJ : public ATSPModel {
public:
	ATSPModel_DFJ(GRBEnv* env, TSPInput& inputData) : ATSPModel(env, inputData) {
		this->modelAlias = "DFJ";
		this->buildModel();
	};
private:
	void buildModel() {
		this->createCstr_DFJSubtourRemoval();
	};
};

class ATSPModel_MTZ : public ATSPModel {
public:
	ATSPModel_MTZ(GRBEnv* env, TSPInput& inputData): ATSPModel(env, inputData) {
		this->modelAlias = "MTZ";
		this->buildModel(); 
	};
private:
	void buildModel() {
		this->createVars_U();
		this->createCstr_MTZSubtourRemoval();
	};
};

class ATSPModel_GG : public ATSPModel {
public:
	ATSPModel_GG(GRBEnv* env, TSPInput& inputData) : ATSPModel(env, inputData) {
		this->modelAlias = "GG";
		this->buildModel();
	};
private:
	void buildModel() {
		this->createVars_G();
		this->createCstr_GVarsFlow();
		this->createConstr_GVarsBoundedByX();
	};
};
