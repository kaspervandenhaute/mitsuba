
#ifndef MITSUBA_OUTLIERDETECTOR
#define MITSUBA_OUTLIERDETECTOR

#include <mitsuba/mitsuba.h>
#include <vector>
#include "../my_pathSeed.h"

MTS_NAMESPACE_BEGIN

class OutlierDetector {

public:

    virtual void contribute(Point2 const& pos, float value) =0;
    virtual float calculateWeight(Point2 const& pos, float value) const =0;
    virtual void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) =0;
    virtual void update(int newSpp) =0;

    inline Point2i discretePosition(Point2 const& pos) const {
        return Point2i(pos.x - 0.5f, pos.y - 0.5f);
    }

};

MTS_NAMESPACE_END

#endif