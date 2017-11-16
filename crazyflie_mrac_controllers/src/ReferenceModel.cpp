#include "ReferenceModel.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <ros/ros.h>
#include <ros/console.h>

using namespace Eigen;
using namespace std;

ReferenceModel::ReferenceModel(void)
{
    this->dt = 0.0;
    this->nStates = 0;
    this->nInputs = 0;
    this->nOutputs = 0;
}
ReferenceModel::~ReferenceModel(void)
{
    
}

void ReferenceModel::initialize(const ReferenceModel_Config_t _config)
{
    // Safety checks
    if (_config.A.cols() != _config.A.rows())
        throw invalid_argument("A is not a squre matrix!");
    if (_config.A.rows() != _config.B.rows())
        throw invalid_argument("A and B have different # of rows!");
    
    this->dt = _config.dt;
    this->Aref = _config.A;
    this->Bref = _config.B;
    this->nStates = this->Aref.cols();
    this->nInputs = this->Bref.cols();
    this->nOutputs = this->nStates; // full states output
    this->Cref = MatrixXd::Identity(this->nOutputs, this->nOutputs);
 
    // Safety checks
    if(_config.states0.size() != this->nStates)
        throw invalid_argument("Unexpected initial states size!");

    this->states0 = VectorXd::Zero(this->nStates);   
    for(int i=0; i<(this->nStates); i++) {
        //ROS_INFO("initialize: %.4f\n", _config.states0(i));
        this->states0(i) = _config.states0(i);
    }
    reset();
}

void ReferenceModel::reset()
{
    this->states = VectorXd::Zero(this->nStates);
    for(int i=0; i<(this->nStates); i++) {

        this->states(i) = this->states0(i);
        //ROS_INFO("Reset: %.4f, ", this->states(i));
    }
    //ROS_INFO("\n");
}

void ReferenceModel::update(const VectorXd _inputs)
{
    // Safety check
    if(_inputs.size() != this->nInputs)
        throw invalid_argument("Unexpected inputs size!");
    //ROS_INFO("update: %.4f", _inputs(0));

    // Euler integration
    VectorXd states_dot = this->Aref * this->states + this->Bref * _inputs;
    this->states += states_dot * this->dt;
}

void ReferenceModel::get_outputs(VectorXd &_outputs)
{
    _outputs = this->Cref * this->states;
}

void ReferenceModel::get_states(VectorXd &_states)
{
    _states = VectorXd::Zero(this->nStates);
    for (int i=0; i<(this->nStates); i++) {
        _states(i) = this->states(i);
    }
    //ROS_INFO("get_states A: %.4f, %.4f", _states(0), _states(1));
    //ROS_INFO("get_states B: %.4f, %.4f", this->states(0), this->states(1));
}
