#ifndef MITSUBA_ZIRR1_H
#define MITSUBA_ZIRR1_H

#include <mitsuba/mitsuba.h>

#include "../my_pathSeed.h"
#include "bitmap3d.h"
#include "outlierDetector.h"

MTS_NAMESPACE_BEGIN

class OutlierDetectorZirr1 : public OutlierDetector {

public:
    OutlierDetectorZirr1(int width, int height, float b, float maxValue, int kappaMin, float threshold);

    void contribute(Point2i const& pos, float value) override;
    float calculateWeight(Point2i const& pos, float value) override;
    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) override;
    void update(int newSpp) override;

private:
    float calcualateOccurencies(Point2i const& pos, float value);
    RatioAndIndex calculateRatioAndIndex(float value);

    const int width, height;
    const float b;
    float maxValue;
    const int kappaMin;
    float threshold;
    const int nbBuffers;
    Bitmap3d<float> buffer, tempBuffer;
    int spp;
    std::vector<float> powersOfb;
    float Emin;
    float cascadeStart = 1;
};

MTS_NAMESPACE_END


#endif //RAY_TRACER_OUTLIERDETECTOR_H