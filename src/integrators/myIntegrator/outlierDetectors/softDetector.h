
#ifndef SOFTDETECTOR_H
#define SOFTDETECTOR_H


class SoftDetector : public OutlierDetector {

public:

    SoftDetector(OutlierDetector* detector, float softness) : 
        OutlierDetector(detector->min, detector->max), 
        detector(detector), 
        random(new Random()), 
        softness(sofness) {}

    virtual void contribute(Point2 const& pos, float value) {
        detector->contribute(pos, value);
    }

    virtual float calculateWeight(Point2 const& pos, float value) const {
        auto weight = detector->calculateWeight(pos, value);

        // If it is an outlier keep it that way
        if (weight == 1.0) {
            return weight;
        }
        // If it is an inlier there is a <softness> chance of returning it as an outlier
        if (random->nextFloat() < softness) {
            return 1;
        }
        return 0;
    }

    virtual void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
        detector->update(seeds, nChains, newSpp);
    }

    virtual void update(int newSpp) {
        detector->update(newSpp);
    }


private:
    ref<OutlierDetector> detector;
    ref<Random> random;
    float softness;
}


#endif