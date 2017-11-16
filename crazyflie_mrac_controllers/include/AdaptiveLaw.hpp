#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

typedef struct {
	MatrixXd Gamma;
	MatrixXd P;
	MatrixXd B;
    double dt;
} AdaptiveLaw_Config_t;

class AdaptiveLaw {
protected:
    double dt;
    MatrixXd BigTheta, Gamma, P, B;
    
public:
    AdaptiveLaw(void);
    ~AdaptiveLaw(void);
    
    void initialize(const AdaptiveLaw_Config_t _config);
    void reset();
    void update(const VectorXd error, const VectorXd  phi);
    void get_gains(MatrixXd &outputs);
};

