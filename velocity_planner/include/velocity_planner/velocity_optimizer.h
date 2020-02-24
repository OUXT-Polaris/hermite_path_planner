#include <ceres/ceres.h>
#include <math.h>
#include <complex>
#include <float.h> // DBL_EPSILON
#include <math.h>  // fabs, fmax

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class VelocityOptimizer
{
private:
    bool SolveCubicEquation(std::complex<double> x[3],double a,double b,double c,double d);
    double Cuberoot(double x);
    std::complex<double> Squreroot(std::complex<double> x);
};

struct AccerelationCost
{
    template <typename T>
    bool operator()(const T* const a,const T* const b, const T* const c, const T* const d,T* residual) const
    {
        residual[0]=0.0;
        return true;
    }
};