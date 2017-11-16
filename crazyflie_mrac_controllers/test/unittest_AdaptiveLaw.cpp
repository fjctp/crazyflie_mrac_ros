 
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "AdaptiveLaw.hpp"

using namespace Eigen;
using namespace std;

int main(void)
{
    cout << "Started" << endl;

    AdaptiveLaw_Config_t config_law_z;
    double dt_sec = 1.0/200.0;

    // Adaptive Law
    config_law_z.Gamma  = 1.0e-3*MatrixXd::Identity(2,2);
    config_law_z.P      = MatrixXd::Zero(2,2);
    config_law_z.B      = MatrixXd::Zero(2,1);
    config_law_z.dt     = dt_sec;
    config_law_z.P(0,0) = 72.373382399897039;
    config_law_z.P(0,1) =  9.231905465288030;
    config_law_z.P(1,0) =  9.231905465288030;
    config_law_z.P(1,1) =  1.384930335176893;
    config_law_z.B(0,0) =  0.0;
    config_law_z.B(1,0) = 26.232948583420782;
    
    cout << config_law_z.P << endl << endl;
    cout << config_law_z.B << endl;
    //return 0;

    AdaptiveLaw law;
    law.initialize(config_law_z);
    
    for(int i=0; i<1000; i++)
    {
        VectorXd errors = VectorXd::Zero(2);
        VectorXd phi = VectorXd::Zero(2);
        if (i >= 200)
        {
            errors(0) = sin(i*dt_sec);
            errors(1) = sin(i*dt_sec-0.1);

            phi(0) = cos(i*dt_sec);
            phi(1) = cos(i*dt_sec-0.1);
        }
        law.update(errors, phi);
        
        MatrixXd outputs;
        law.get_gains(outputs);
        
        cout << i << "," << outputs(0) << endl;
    }
    return 0;
}
