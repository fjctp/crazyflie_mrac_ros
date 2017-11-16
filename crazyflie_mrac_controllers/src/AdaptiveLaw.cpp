#include "AdaptiveLaw.hpp"
#include <Eigen/Dense>
#include <stdexcept>

using namespace Eigen;
using namespace std;

AdaptiveLaw::AdaptiveLaw(void)
{
    this->dt = 0.0;
}
AdaptiveLaw::~AdaptiveLaw(void)
{
    
}

void AdaptiveLaw::initialize(const AdaptiveLaw_Config_t _config)
{
    this->Gamma = _config.Gamma;
    this->P = _config.P;
    this->B = _config.B;
    this->dt = _config.dt;
    
    reset();
}

void AdaptiveLaw::reset()
{
	this->BigTheta = MatrixXd::Zero(this->Gamma.rows(), this->B.cols());
}

void AdaptiveLaw::update(const VectorXd error, const VectorXd  phi)
{
    MatrixXd BigTheta_dot = this->Gamma * phi * error.transpose() * this->P * this->B;
    this->BigTheta += BigTheta_dot * this->dt;
}

void AdaptiveLaw::get_gains(MatrixXd &outputs)
{
    outputs = this->BigTheta;
}
