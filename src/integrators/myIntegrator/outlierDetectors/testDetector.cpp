
#include "testDetector.h"

MTS_NAMESPACE_BEGIN

void TestOutlierDetector::contribute(Point2 const& pos, float value) {}
float TestOutlierDetector::calculateWeight(Point2 const& posFloat, float value) const  {
    // auto pos = discretePosition(posFloat);

    // if (pos.x % 2 == 0) {
    //     return pos.y % 2 == 0;
    // } else {
    //     return pos.y % 2 != 0;
    // }

    // return value > 2.f ? 1.f : 0.f;

    // return pos.x == 0 && pos.y == 0  ? 1.f : 0.f;
    return 1.f;

}

void TestOutlierDetector::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {}
void TestOutlierDetector::update(int newSpp) {}


MTS_NAMESPACE_END

