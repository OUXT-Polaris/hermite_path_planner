#include <velocity_planner/velocity_optimizer.h>

bool VelocityOptimizer::SolveCubicEquation(std::complex<double> x[3],double a,double b,double c,double d)
{
    if(std::fabs(a)<DBL_EPSILON)
    {
        return false;
    }
    double A,B,C,p,q,D;
    A = b/a;
    B = c/a;
    C = d/a;
    p = B-A*A/3.0;
    q = 2.0*A*A*A/27.0-A*B/3.0+C;
    D = q*q/4.0+p*p*p/27.0;
    if(D < 0.0)
    {//three real solutions
        double theta = atan2(sqrt(-D),-q*0.5);
        x[0] = 2.0*sqrt(-p/3.0)*cos(theta/3.0)-A/3.0;
        x[1] = 2.0*sqrt(-p/3.0)*cos((theta+2.0*M_PI)/3.0)-A/3.0;
        x[2] = 2.0*sqrt(-p/3.0)*cos((theta+4.0*M_PI)/3.0)-A/3.0;
    }
    else
    {//single real solution and two imaginary solutions(c.c)
        std::complex<double> i(0.0,1.0);
        double u = Cuberoot(-q*0.5+sqrt(D)),v = Cuberoot(-q*0.5-sqrt(D));
        x[0] = u+v-A/3.0;
        x[1] = -0.5*(u+v)+sqrt(3.0)*0.5*i*(u-v)-A/3.0;
        x[2] = -0.5*(u+v)-sqrt(3.0)*0.5*i*(u-v)-A/3.0;
    }
    return true;
}

double VelocityOptimizer::Cuberoot(double x)
{
    if(x > 0.0)
    {
        return pow(x,1.0/3.0);
    }
    else
    {
        return -pow(-x,1.0/3.0);
    }
}

std::complex<double> VelocityOptimizer::Squreroot(std::complex<double> x){
    std::complex<double> y;
    std::complex<double> i(0.0,1.0);
    double r = sqrt(x.real()*x.real()+x.imag()*x.imag()),theta = atan2(x.imag(),x.real());
    if(x.imag() == 0.0)
    {
        if(x.real() > 0.0)
        {
            y = sqrt(r);
        }
        else
        {
            y = sqrt(r)*i;
        }
    }
    else
    {
        if(theta < 0.0)
        {
            theta += 2.0*M_PI;
        }
        y = sqrt(r)*(cos(theta*0.5)+i*sin(theta*0.5));
    }
    return y;
}