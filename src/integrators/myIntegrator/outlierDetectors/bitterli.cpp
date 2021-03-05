
#include <cmath>
#include "bitterli.h"


MTS_NAMESPACE_BEGIN

OutlierDetectorBitterly::OutlierDetectorBitterly(int width, int height, int nbBuffers1, float alfa, float beta, float maxValue) :
        width(width), height(height), nbBuffers(7/*TODO What the FUCK?!! */), alfaInv(1/alfa), beta(beta), maxValue(maxValue), minValue(0.5),
        buffer(width, height, nbBuffers1), tempBuffer(width, height, nbBuffers1), spp(0) {
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

        float result = buffer.get(pos.x, pos.y, ratioAndIndex.index) * (1-ratioAndIndex.ratio);
        if (ratioAndIndex.index +1 < nbBuffers) {
            result += buffer.get(pos.x, pos.y, ratioAndIndex.index+1) *   ratioAndIndex.ratio;
        }
        return result;
    }
    return 0;
}   

float OutlierDetectorBitterly::calculateWeight(Point2i const& pos, float value) {
    assert(pos.y < height && pos.x < width);

    if (value > maxValue) {
        return 1;
    } else if (value < minValue) {
        return 0;
    }

    float threshold = calculateThreshold(pos);
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

float OutlierDetectorBitterly::calculateThreshold(Point2i const& pos) {
    return std::max(minThreshold, ((float) spp)/nbBuffers);
}

void OutlierDetectorBitterly::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
    buffer.add(tempBuffer);
    tempBuffer.reset();
    setAdditionalThreshold(seeds, nChains);
    spp = newSpp;
}

void OutlierDetectorBitterly::setAdditionalThreshold(std::vector<PositionedPathSeed> const& allSeeds, size_t nChains) {

    std::vector<float> probs;
    probs.reserve(allSeeds.size());

    float sum = 0;
    for (auto& seed : allSeeds) {
        sum += seed.luminance;
    }
    auto normalization = 1.f/sum;

    PositionedPathSeed const* closestSeed = nullptr;
    float closestDistance = std::numeric_limits<float>::infinity();
    for (auto& seed : allSeeds) {
        auto probNotPicked = std::pow(1.f - seed.luminance * normalization, nChains);
        auto dist = probNotPicked - 0.5;
        if (dist > 0 && dist < closestDistance) {
            closestDistance = dist;
            closestSeed = &seed;
        }
    }
    
    if (closestDistance != std::numeric_limits<float>::infinity()) {
        minValue = closestSeed->luminance;
    }

}


MTS_NAMESPACE_END
