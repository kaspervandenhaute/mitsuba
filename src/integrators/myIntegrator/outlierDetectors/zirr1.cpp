
#include "zirr1.h"

MTS_NAMESPACE_BEGIN


OutlierDetectorZirr1::OutlierDetectorZirr1(int width, int height, float b, float maxValue, int kappaMin, float threshold) :
        width(width), height(height), b(b), maxValue(maxValue), kappaMin(kappaMin), threshold(threshold), 
        nbBuffers(std::ceil(std::log(maxValue))), buffer(width, height, nbBuffers), tempBuffer(width, height, nbBuffers), spp(0), powersOfb(nbBuffers) {
        std::cout << nbBuffers << "  " << std::ceil(std::log(maxValue)) << std::endl;
        powersOfb[0] = b;
        for (int i=1; i<nbBuffers; ++i) {
            powersOfb[i] = powersOfb[i-1] * b;
        }
}

void OutlierDetectorZirr1::contribute(Point2i const& pos, float value) {
    assert(pos.y < height && pos.x < width);

    // If the value of the contribution is bigger than max_value we ignore it.
    if (value < maxValue) {
        auto ratioAndIndex = calculateRatioAndIndex(value);

        tempBuffer.add(pos.x, pos.y, ratioAndIndex.index, value*ratioAndIndex.ratio);
        if (ratioAndIndex.index +1 < nbBuffers) {
            tempBuffer.add(pos.x, pos.y, ratioAndIndex.index+1,  value*(1-ratioAndIndex.ratio));
        }
    }
}

float OutlierDetectorZirr1::calcualateOccurencies(Point2i const& pos, float value) {
    assert(pos.y < height && pos.x < width);

    if (value < maxValue) {
        auto ratioAndIndex = calculateRatioAndIndex(value);
        auto j = ratioAndIndex.index;
        
        float result = 0;
        for (int i=-1; i<=1; ++i) {
            if (j + i < nbBuffers && j+i > 0) {
                for (int x0=-1; x0<=1; ++x0) {
                    for (int y0=-1; y0<=1; ++y0) {
                        auto x = pos.x + x0;
                        auto y = pos.y + y0;
                        if (x > 0 && x < width && y > 0 && y < height) {
                            result += spp * buffer.get(pos.x, pos.y, j+i) / powersOfb[j+i];  
                        }
                    }
                }
                
            }
        }
        return result;
    }
    return 0;
}   

float OutlierDetectorZirr1::calculateWeight(Point2i const& pos, float value) {
    assert(pos.y < height && pos.x < width);

    if (value > maxValue) {
        return 1;
    }

    float occurencies = calcualateOccurencies(pos, value);

    if (occurencies < kappaMin) {
        return 1;
    }
    // auto rStar = spp/(occurencies-kappaMin);
    // if (rStar > threshold) {
    //     return 1;
    // }

    return 0;
}

RatioAndIndex OutlierDetectorZirr1::calculateRatioAndIndex(float value) {
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
    // assert(ratio >= 0 && ratio <= 1);
    assert(j < nbBuffers);
    return {ratio, j};
}

void OutlierDetectorZirr1::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
    buffer.add(tempBuffer);
    tempBuffer.reset();
    spp = newSpp;
}


MTS_NAMESPACE_END