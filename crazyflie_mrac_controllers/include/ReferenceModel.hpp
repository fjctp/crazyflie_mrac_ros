#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

typedef struct {
    MatrixXd A;
    MatrixXd B;
    VectorXd states0;
    double dt;
} ReferenceModel_Config_t;

class ReferenceModel {
protected:
    double dt;
    unsigned int nStates, nInputs, nOutputs;
    
    VectorXd states0;
    VectorXd states;
    MatrixXd Aref, Bref, Cref;
    
public:
    ReferenceModel(void);
    ~ReferenceModel(void);
    
    void initialize(const ReferenceModel_Config_t _config);
    void reset();
    void update(const VectorXd _inputs);
    void get_outputs(VectorXd &_outputs);
    void get_states(VectorXd &_states);
};

