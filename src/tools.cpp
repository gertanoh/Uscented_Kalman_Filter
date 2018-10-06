#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,	const vector<VectorXd> &ground_truth) 
{
  
	
	// 2-d only 
	
	VectorXd rmse(4);
	rmse << 0.0, 0.0, 0.0, 0.0;
	
	/* check for incorrect size of input */
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
	{
		std::cout <<"Incompatible size of estimations" << std::endl;
		return rmse;
	}
	
	for (size_t i = 0; i < estimations.size(); ++i)
	{
		
		VectorXd v = estimations[i] - ground_truth[i];
		v = v.array() * v.array();
		
		rmse += v;
	}
	
	rmse /= estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;

}


