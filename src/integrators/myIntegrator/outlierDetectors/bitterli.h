
#ifndef MITSUBA_OUTLIERDETECTORBITTERLI_H
#define MITSUBA_OUTLIERDETECTORBITTERLI_H

#include <mitsuba/mitsuba.h>

#include <vector>

#include "../my_pathSeed.h"
#include "bitmap3d.h"
#include "outlierDetector.h"

MTS_NAMESPACE_BEGIN

class OutlierDetectorBitterly : public OutlierDetector {

public:
    OutlierDetectorBitterly(int width, int height, int nbBuffers, float alfa, float beta, float maxValue);

    void contribute(Point2 const& pos, float value) override;
    float calculateWeight(Point2 const& pos, float value) const override;
    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) override;
    void update(int newSpp) override;


private:
    const int width, height;
    const int nbBuffers;
    const float alfaInv, beta;
    Bitmap3d<float> buffer, tempBuffer;
    float minThreshold = 3;
    int spp;


    float calcualateOccurencies(Point2i const& pos, float value) const;
    float calculateThreshold(Point2i const& pos) const;
    RatioAndIndex calculateRatioAndIndex(float value) const;
    void setAdditionalThreshold(std::vector<PositionedPathSeed> const& seeds, size_t nChains);

};

MTS_NAMESPACE_END


#endif //RAY_TRACER_OUTLIERDETECTOR_H
