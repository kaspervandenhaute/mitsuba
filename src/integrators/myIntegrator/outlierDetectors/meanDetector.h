
#ifndef MEANDETECTOR_H
#define MEANDETECTOR_H

#include <mitsuba/mitsuba.h>

#include "outlierDetector.h"

MTS_NAMESPACE_BEGIN

class MeanOutlierDetector : public OutlierDetector {

public:

    MeanOutlierDetector(int width, int height, int total_spp, float threshold) : 
        OutlierDetector(0,0), 
        height(height), width(width), 
        sums(width*height, 0.f), sums_temp(width*height, 0.f), 
        counts(width*height, 0), counts_temp(width*height, 0), 
        total_spp{total_spp},
        threshold(threshold), current_spp(0) {}

    void contribute(Point2 const& pos, float value) override;
    inline void contributeMlt(Point2 const& pos, float value) override {contribute(pos, value);}
    inline float calculateWeight(Point2 const& posFloat, float value) const override;

    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) override;
    void update(int newSpp) override;

private:

    float calculateWeightIndex(const int index, const float value) const;

    int get_index(Point2 const& posFloat) const {
        auto pos = discretePosition(posFloat);
        if (pos.y == height) {
            pos.y -= 1;
        }
        if (pos.x == width) {
            pos.x -= 1;
        }
        assert(pos.y < height && pos.x < width);

        return pos.x + width * pos.y;
    }

    int height, width;
    std::vector<float> sums, sums_temp;
    std::vector<uint32_t> counts, counts_temp;
    int total_spp;
    float threshold;
    uint32_t current_spp;
    int last_index = -1;
    std::vector<float> init_samples;
};

MTS_NAMESPACE_END

#endif