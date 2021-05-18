
#ifndef MEANDETECTOR_H
#define MEANDETECTOR_H

#include <mitsuba/mitsuba.h>

#include "outlierDetector.h"

#include <atomic>

MTS_NAMESPACE_BEGIN

class MeanOutlierDetector : public OutlierDetector {

public:

    MeanOutlierDetector(int width, int height, int total_spp, float threshold) : 
        OutlierDetector(0,0), 
        height(height), width(width), 
        current_mean(width*height, 0.f), 
        mlt_sums_temp(width*height, 0.f), 
        sums_temp(width*height, 0.f), 
        total_spp{total_spp},
        threshold(threshold), current_spp(0), mlt_samples(0) {}

    void contribute(Point2 const& pos, float value) override;
    inline void contributeMlt(Point2 const& pos, float value) override;

    void init(Point2i const& pos, const float value);

    inline float calculateWeight(Point2 const& posFloat, float value) const override;

    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) override;
    void update(int newSpp) override;

private:

    float calculateWeightDiscrete(Point2i const& pos, const float value) const;

    inline Point2i discrete_pos(Point2 const& posFloat) const {
        auto pos = discretePosition(posFloat);
        if (rintf(pos.y) == pos.y && pos.y != 0.f) {
            pos.y -= 1;
        }
        if (rintf(pos.x) == pos.x && pos.x != 0.f) {
            pos.x -= 1;
        }
        assert(pos.y < height && pos.x < width);
        assert(pos.y >= 0.f && pos.x >= 0.f);

        return pos;
    }

    inline int get_index(Point2i const& pos) const {
        return get_index(pos.x, pos.y);
    }

    inline int get_index(const int x, const int y) const {
        return x + width * y;
    }

    float neighbors_mean(Point2i const& pos, const int r=1) const;



    int height, width;
    std::vector<float> current_mean;
    std::vector<float> mlt_sums_temp, sums_temp;
    int total_spp;
    float threshold;
    uint32_t current_spp;
    std::atomic<uint32_t> mlt_samples;
    int iteration = 0;
};

MTS_NAMESPACE_END

#endif