#pragma once

#include "models.hpp"
#include "exceptions.hpp"
#include "inputOutput.hpp"

class TSPCreator {
public:
	TSPCreator(GRBEnv* env, TSPInput data) {
		this->env = env;
		this->data = data;
	};

	ATSPModel* build(ATSP_ModelType inputModel);
private:
	GRBEnv* env;
	TSPInput data;
};

ATSPModel* TSPCreator::build(ATSP_ModelType inputModel) {
	ATSPModel* model{};
	switch (inputModel){
	case ATSP_ModelType::DFJ:
		model = new ATSPModel_DFJ(this->env, this->data);
		return model;
		break;
	case ATSP_ModelType::MTZ:
		model = new ATSPModel_MTZ(this->env, this->data);
		return model;
		break;
	case ATSP_ModelType::GG:
		model = new ATSPModel_GG(this->env, this->data);
		return model;
		break;
	default:
		throw TSPFactoryBadInput();
		break;
	}
}
