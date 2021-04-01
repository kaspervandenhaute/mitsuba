
#ifndef MITSUBA_THRESHHOLDOUTLIERDETECTOR
#define MITSUBA_THRESHHOLDOUTLIERDETECTOR

#include "outlierDetector.h"

MTS_NAMESPACE_BEGIN

struct ExponentialAverage {
    ExponentialAverage(float t) : t(t), avg(0) {}

    inline void add(float val) {
        if (avg == 0) {
            avg = val;
        } else {
            avg = (1-t) * avg + t * val;
        }
    }

    inline float get() const {
        return avg;
    }

    const float t;
    float avg;
};

class ThresholdDetector : public OutlierDetector {

public:

    ThresholdDetector() : OutlierDetector(0,0) {}

    void contribute(Point2 const& pos, float value) override {}

    float calculateWeight(Point2 const& pos, float value) const override {
        return value > minValue;
    }

    virtual void update(std::vector<PositionedPathSeed> const& allSeeds, size_t nChains, int newSpp) {

        std::vector<float> probs;
        probs.reserve(allSeeds.size());

        float sum = 0;
        for (auto& seed : allSeeds) {
            sum += seed.luminance;
        }
        auto normalization = 1.f/sum;

        PositionedPathSeed const* closestSeed = nullptr;
        float closestDistance = std::numeric_limits<float>::infinity();
        for (auto& seed : allSeeds) {
            auto probNotPicked = std::pow(1.f - seed.luminance * normalization, nChains);
            auto dist = probNotPicked - 0.5;
            if (dist > 0 && dist < closestDistance) {
                closestDistance = dist;
                closestSeed = &seed;
            }
        }
        
        if (closestDistance != std::numeric_limits<float>::infinity()) {
            minValue = closestSeed->luminance;
        }
    }

    virtual void update(int newSpp) {}

};

MTS_NAMESPACE_END

#endif