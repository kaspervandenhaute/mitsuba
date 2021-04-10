
#ifndef TESTDETECTOR_H
#define TESTDETECTOR_H

#include <mitsuba/mitsuba.h>

#include "outlierDetector.h"

MTS_NAMESPACE_BEGIN

class TestOutlierDetector : public OutlierDetector {

public:

    TestOutlierDetector() : OutlierDetector(0,0) {}

    void contribute(Point2 const& pos, float value) override;
    inline float calculateWeight(Point2 const& posFloat, float value) const override;

    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) override;
    void update(int newSpp) override;

};

MTS_NAMESPACE_END

#endif
