
#include <cmath>
#include "bitterli.h"

MTS_NAMESPACE_BEGIN

OutlierDetectorBitterly::OutlierDetectorBitterly(int width, int height, int nbBuffers1, float alfa, float beta, float maxValue) :
        width(width), height(height), nbBuffers(7/* What the FUCK?!! */), alfaInv(1/alfa), beta(beta), maxValue(maxValue), minValue(0.5),
        buffer(width, height, nbBuffers1), tempBuffer(width, height, nbBuffers1) {
        std::cout << nbBuffers << std::endl;
}

void OutlierDetectorBitterly::contribute(Point2i const& pos, float value) {
    assert(pos.y < height && pos.x < width);

    // If the value of the contribution is bigger than max_value we ignore it.
    if (value < maxValue) {
        auto ratioAndIndex = calculateRatioAndIndex(value);

        tempBuffer.add(pos.x, pos.y, ratioAndIndex.index, 1- ratioAndIndex.ratio);
        if (ratioAndIndex.index +1 < nbBuffers) {
            tempBuffer.add(pos.x, pos.y, ratioAndIndex.index+1,  ratioAndIndex.ratio);
        }
    }
}

float OutlierDetectorBitterly::calcualateOccurencies(Point2i const& pos, float value) {
    assert(pos.y < height && pos.x < width);

    if (value < maxValue) {
        auto ratioAndIndex = calculateRatioAndIndex(value);

        float result = buffer.get(pos.x, pos.y, ratioAndIndex.index)   * (1-ratioAndIndex.ratio);
        if (ratioAndIndex.index +1 < nbBuffers) {
            result += buffer.get(pos.x, pos.y, ratioAndIndex.index+1) *    ratioAndIndex.ratio;
        }
        return result;
    }
    return 0;
}   

float OutlierDetectorBitterly::calculateWeight(Point2i const& pos, float value, int spp) {
    assert(pos.y < height && pos.x < width);

    if (value > maxValue) {
        return 1;
    } else if (value < minValue) {
        return 0;
    }

    float threshold = calculateThreshold(pos, spp);
    float occurencies = calcualateOccurencies(pos, value);

    if (occurencies < threshold) {
        return 1;
    }
    return 0;
}

RatioAndIndex OutlierDetectorBitterly::calculateRatioAndIndex(float value) {
    float logVal = std::log(value * alfaInv) / std::log(beta); //TODO: store 1/log(beta)
    int j = std::floor(logVal);

    float ratio;
    
    if (j >= nbBuffers-1) {
        // std::cout << "J is: " << j << "  nbBuffers: " << nbBuffers << std::endl;
        j = nbBuffers-1;
        ratio = 0;
    } else if (j < 0) {
        j = 0;
        ratio = 0;
    } else {
        ratio = logVal - (float) j;
    }
    // std::cout << "J is: " << j << "  nbBuffers: " << nbBuffers << std::endl;


    assert(ratio >= 0 && ratio <= 1);

    return {ratio, j};
}

float OutlierDetectorBitterly::calculateThreshold(Point2i const& pos, int spp) {
    return std::max(minThreshold, ((float) spp)/nbBuffers);
}

void OutlierDetectorBitterly::startIteration() {
    buffer.add(tempBuffer);
    tempBuffer.reset();
    setAdditionalThreshold();
}

void OutlierDetectorBitterly::setAdditionalThreshold() {
    
}


MTS_NAMESPACE_END