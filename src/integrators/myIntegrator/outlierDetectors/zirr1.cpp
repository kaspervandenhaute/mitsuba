
#include "zirr1.h"

#include <mitsuba/core/statistics.h>

MTS_NAMESPACE_BEGIN

StatsCounter rcCounter("Zirr1", "Fraction of outliers because of r*c", EPercentage);


OutlierDetectorZirr1::OutlierDetectorZirr1(int width, int height, float b, float maxValue, int kappaMin, float threshold) :
        OutlierDetector(0.5, maxValue), width(width), height(height), b(b), kappaMin(kappaMin), threshold(threshold), 
        nbBuffers(std::ceil(std::log(maxValue)/std::log(b))), buffer(width, height, nbBuffers), tempBuffer(width, height, nbBuffers), spp(0), powersOfb(nbBuffers) {
        std::cout << nbBuffers << "  " << std::ceil(std::log(maxValue)) << std::endl;
        powersOfb[0] = 1;
        for (int i=1; i<nbBuffers; ++i) {
            powersOfb[i] = powersOfb[i-1] * b;
        }
}

void OutlierDetectorZirr1::contribute(Point2 const& posFloat, float value) {
    auto pos = discretePosition(posFloat);
    assert(pos.y < height && pos.x < width);

// If the value of the contribution is bigger than max_value we ignore it.
    if (value < maxValue) {

        float lowerScale = cascadeStart;
        float upperScale = lowerScale * b;
        int baseIndex = 0;

        /* find adjacent layers in cascade for <luminance> */
        while (!(value < upperScale) && baseIndex < nbBuffers - 2) {
            lowerScale = upperScale;
            upperScale *= b;
            ++baseIndex;
        }

        /* weight for lower buffer */
        float weightLower;
        if (value <= lowerScale)
            weightLower = 1.0f;
        else if (value < upperScale)
            weightLower = std::max( 0.0f,
                (lowerScale / value - lowerScale / upperScale) / (1 - lowerScale / upperScale) );
        else // Inf, NaN ...
            weightLower = 0.0f;
        
        /* weight for higher buffer */
        float weightUpper;
        if (value < upperScale)
            weightUpper = std::max(0.0f, 1 - weightLower);
        else // out of cascade range, we don't expect this to converge
            weightUpper = upperScale / value;
    

        tempBuffer.add(pos.x, pos.y, baseIndex, value*weightLower);
        if (baseIndex +1 < nbBuffers) {
            tempBuffer.add(pos.x, pos.y, baseIndex+1,  value*weightUpper);
        }
    }
}

float OutlierDetectorZirr1::calcualateOccurencies(Point2i const& pos, int j) const {
    assert(pos.y < height && pos.x < width);
    
    float result = 0;
    for (int i=-1; i<=1; ++i) {
        if (j + i < nbBuffers && j+i >= 0) {
            for (int x0=-1; x0<=1; ++x0) {
                for (int y0=-1; y0<=1; ++y0) {
                    auto x = pos.x + x0;
                    auto y = pos.y + y0;
                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        result += spp * buffer.get(pos.x, pos.y, j+i) / powersOfb[j+i];  
                    }
                }      
            }
            
        }
    }
    return result;

}   

float OutlierDetectorZirr1::calculateWeight(Point2 const& posFloat, float value) const {
    auto pos = discretePosition(posFloat);
    assert(pos.y < height && pos.x < width);

    if (value < 0.0000001) {
        return 0;
    }

    if (value > maxValue) {
        return 1;
    }

    auto ratioAndIndex = calculateRatioAndIndex(value);
    int index = ratioAndIndex.index;
    
    float Emin = 0;
    for (int j=0; j<=index; j++) {
        Emin += calcualateOccurencies(pos, j) / spp;
    }

    auto rStarV = powersOfb[index] / Emin;

    auto rStarC = spp/(calcualateOccurencies(pos, index) - kappaMin);
    if (rStarC < 0) {
        rStarC = 1;
    }

    // std::cout << "r*c: " << rStarC << "  r*v: " << rStarV << std::endl;
    
    if (std::min(rStarC, rStarV) > threshold) {
        if (rStarC < rStarV) {
            ++rcCounter;
        } 
        rcCounter.incrementBase(1);
        return 1;
    }

    return 0;
}

RatioAndIndex OutlierDetectorZirr1::calculateRatioAndIndex(float value) const {
    int j = std::floor(std::log(value) / std::log(b)); //TODO: store 1/log(beta)

    float ratio;
    
    if (j < 0) {
        j = 0;
        ratio = 1;
    } else {
        auto invb = 1.f/b;
        ratio = (powersOfb[j]/value - invb) / (1-invb);
    }
    // std::cout << "J is: " << j << "  nbBuffers: " << nbBuffers << std::endl;
    assert(ratio >= 0 && ratio <= 1);
    assert(j < nbBuffers);
    return {ratio, j};
}

void OutlierDetectorZirr1::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
    update(newSpp);
}

void OutlierDetectorZirr1::update(int newSpp) {
    buffer.add(tempBuffer);
    tempBuffer.reset();
    spp = newSpp;
}



MTS_NAMESPACE_END