#ifndef RUNNINGSTATS_H
#define RUNNINGSTATS_H

// https://www.johndcook.com/blog/skewness_kurtosis/

class RunningStats
{
public:
    RunningStats();
    void Clear();
    void Push(double x);
    long long NumDataValues() const;
    float Mean() const;
    float Variance() const;
    float StandardDeviation() const;
    float Skewness() const;
    float Kurtosis() const;

    friend RunningStats operator+(const RunningStats a, const RunningStats b);
    RunningStats& operator+=(const RunningStats &rhs);

private:
    long long n;
    float M1, M2, M3, M4;
};

#endif