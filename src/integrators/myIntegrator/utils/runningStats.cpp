#include "runningStats.h"
#include <math.h>

RunningStats::RunningStats() 
{
    Clear();
}

void RunningStats::Clear()
{
    n = 0;
    M1 = M2 = M3 = M4 = 0.0;
}

void RunningStats::Push(double x)
{
    double delta, delta_n, delta_n2, term1;

    long long n1 = n;
    n++;
    delta = x - M1;
    delta_n = delta / n;
    delta_n2 = delta_n * delta_n;
    term1 = delta * delta_n * n1;
    M1 += delta_n;
    M4 += term1 * delta_n2 * (n*n - 3*n + 3) + 6 * delta_n2 * M2 - 4 * delta_n * M3;
    M3 += term1 * delta_n * (n - 2) - 3 * delta_n * M2;
    M2 += term1;
}

long long RunningStats::NumDataValues() const
{
    return n;
}

float RunningStats::Mean() const
{
    return M1;
}

float RunningStats::Variance() const
{
    return M2/(n-1.0);
}

float RunningStats::StandardDeviation() const
{
    return std::sqrt( Variance() );
}

float RunningStats::Skewness() const
{
    return std::sqrt(float(n)) * M3/ std::pow(M2, 1.5);
}

float RunningStats::Kurtosis() const
{
    return float(n)*M4 / (M2*M2) - 3.0;
}

RunningStats& RunningStats::operator+=(const RunningStats& other)
{ 
    
    n = n + other.n;

    if (n > 0) {
    
        float delta = other.M1 - M1;
        float delta2 = delta*delta;
        float delta3 = delta*delta2;
        float delta4 = delta2*delta2;
        
        M1 = (n*M1 + other.n*other.M1) / n;
        
        M2 = M2 + other.M2 + delta2 * n * other.n / n;
        
        M3 = M3 + other.M3 + delta3 * n * other.n * (n - other.n)/(n*n);
        M3 += 3.0*delta * (n*other.M2 - other.n*M2) / n;
        
        M4 = M4 + other.M4 + delta4*n*other.n * (n*n - n*other.n + other.n*other.n) / 
                    (n*n*n);
        M4 += 6.0*delta2 * (n*n*other.M2 + other.n*other.n*M2)/(n*n) + 
                    4.0*delta*(n*other.M3 - other.n*M3) / n;

    }
    
    return *this;
}