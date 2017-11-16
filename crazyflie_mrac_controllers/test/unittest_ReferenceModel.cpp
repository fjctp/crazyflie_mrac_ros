#include <iostream>
#include <Eigen/Dense>
#include "ReferenceModel.hpp"

using namespace Eigen;
using namespace std;

int main(void)
{
    cout << "Started" << endl;
    ReferenceModel_Config_t config_model_z;
    // Reference model
    config_model_z.A        = MatrixXd::Zero(2,2);
    config_model_z.B        = MatrixXd::Zero(2,1);
    config_model_z.states0  = VectorXd::Zero(2);
    config_model_z.dt       = 1.0/200.0;
    config_model_z.A(0,0)   =  0.0;
    config_model_z.A(0,1)   =  1.0;
    config_model_z.A(1,0)   = -5.416;
    config_model_z.A(1,1)   = -7.027;
    config_model_z.B(0,0)   = -0.1145;
    config_model_z.B(1,0)   =  6.273;

    cout << config_model_z.A;

    ReferenceModel model;
    model.initialize(config_model_z);

    for(int i=0; i<1000; i++)
    {
        if (i >= 200)
        {
            VectorXd inputs = VectorXd::Zero(1);
            inputs(0) = -0.5;
            model.update(inputs);
        }
        else
            model.update(VectorXd::Zero(1));
        
        VectorXd outputs;
        model.get_outputs(outputs);
        
        cout << i << "," << outputs(0) << endl;
        
    }
    return 0;
}
