
#ifndef MITSUBA_OUTLIERDETECTOR
#define MITSUBA_OUTLIERDETECTOR

#include <mitsuba/mitsuba.h>
#include <vector>
#include "../my_pathSeed.h"

MTS_NAMESPACE_BEGIN

enum Outlier {
    EInlier, EOutlier
};

class OutlierDetector {

public:

    OutlierDetector(float min, float max) : minValue(min), maxValue(max) {}

    virtual void contribute(Point2 const& pos, float value) =0;
    virtual void contributeMlt(Point2 const& pos, float value) {}
    virtual float calculateWeight(Point2 const& pos, float value) const =0;
    virtual void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) =0;
    virtual void update(int newSpp) =0;

    inline Point2i discretePosition(Point2 const& pos) const {
        // return Point2i(pos.x - 0.5f, pos.y - 0.5f);
        // return Point2i(std::ceil(pos.x - 0.5f), std::ceil(pos.y - 0.5f));
        return Point2i(std::floor(pos.x), std::floor(pos.y));
    }

    float minValue, maxValue;

};

MTS_NAMESPACE_END

#endif