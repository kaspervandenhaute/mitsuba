
#ifndef SOFTDETECTOR_H
#define SOFTDETECTOR_H

#include <memory>

#include "outlierDetector.h"
#include <mitsuba/core/random.h>

MTS_NAMESPACE_BEGIN

class SoftDetector {

public:

    SoftDetector(std::unique_ptr<OutlierDetector> detector, float softness) : 
        detector(std::move(detector)), 
        softness(softness) {}

    inline void contribute(Point2 const& pos, float value) {
        detector->contribute(pos, value);
    }

    inline void contributeMlt(Point2 const& pos, float value) {
        detector->contributeMlt(pos, value);
    }

    inline float calculateWeight(Point2 const& pos, float value, float rand) const {
        auto weight = detector->calculateWeight(pos, value);

        // If it is an outlier keep it that way
        if (weight == 1.0) {
            return weight;
        }
        // If it is an inlier there is a <softness> chance of returning it as an outlier
        if (rand < softness) {
            return 1;
        }
        return 0;
    }

    inline void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
        detector->update(seeds, nChains, newSpp);
    }

    inline void update(int newSpp) {
        detector->update(newSpp);
    }

    inline float getMin() const {
        return detector->minValue;
    }

    inline float getMax() const {
        return detector->maxValue;
    }


private:
    std::unique_ptr<OutlierDetector> detector;
    float softness;
};

MTS_NAMESPACE_END

#endif