#include <iostream>
#include "Dense"
using namespace Eigen;
//Evaluate a polynomial

double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0 ;
    for (int i = 0; i< coeffs.size(); i++)
    {
        result += coeffs[i]*pow(x,i);
    }
    return result;
}

//Fit a polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); i++)
    {
        A(i,0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for(int i = 0; i < order; i++)
        {
            A(j, i+1) = A(j, i)*xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}