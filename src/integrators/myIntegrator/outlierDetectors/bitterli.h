
#ifndef MITSUBA_OUTLIERDETECTOR_H
#define MITSUBA_OUTLIERDETECTOR_H

#include <mitsuba/mitsuba.h>

#include <vector>

#include "../my_pathSeed.h"
#include "bitmap3d.h"

MTS_NAMESPACE_BEGIN

class OutlierDetectorBitterly {

public:
    OutlierDetectorBitterly(int width, int height, int nbBuffers, float alfa, float beta, float maxValue);

    void contribute(Point2i const& pos, float value);
    float calculateWeight(Point2i const& pos, float value);
    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp);
    void update(int newSpp);

private:
    const int width, height;
    const int nbBuffers;
    const float alfaInv, beta;
    float maxValue, minValue;
    Bitmap3d<float> buffer, tempBuffer;
    float minThreshold = 3;
    int spp;


    float calcualateOccurencies(Point2i const& pos, float value);
    float calculateThreshold(Point2i const& pos);
    RatioAndIndex calculateRatioAndIndex(float value);
    void setAdditionalThreshold(std::vector<PositionedPathSeed> const& seeds, size_t nChains);

};

MTS_NAMESPACE_END


#endif //RAY_TRACER_OUTLIERDETECTOR_H
