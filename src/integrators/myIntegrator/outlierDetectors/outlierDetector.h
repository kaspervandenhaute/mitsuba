
#ifndef MITSUBA_OUTLIERDETECTOR
#define MITSUBA_OUTLIERDETECTOR



MTS_NAMESPACE_BEGIN

class OutlierDetector {

public:

    virtual void contribute(Point2 const& pos, float value) =0;
    virtual float calculateWeight(Point2 const& pos, float value) const =0;
    virtual void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) =0;
    virtual void update(int newSpp) =0;

};

MTS_NAMESPACE_END

#endif