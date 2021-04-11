
#include "zirr1.h"

#include <mitsuba/core/statistics.h>

MTS_NAMESPACE_BEGIN

float sumReliabilities = 0;
int64_t countReliabilities = 0;


OutlierDetectorZirr1::OutlierDetectorZirr1(int width, int height, float b, float maxValue, int kappa, float threshold) :
        OutlierDetector(0.5, maxValue), width(width), height(height), b(b), oneOverK(1.f/kappa), threshold(threshold), 
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
                (lowerScale / value - lowerScale / upperScale) / (1.f - lowerScale / upperScale) );
        else // Inf, NaN ...
            weightLower = 0.0f;
        
        /* weight for higher buffer */
        float weightUpper;
        if (value < upperScale)
            weightUpper = std::max(0.0f, 1.f - weightLower);
        else // out of cascade range, we don't expect this to converge
            weightUpper = upperScale / value;
    

        tempBuffer.add(pos.x, pos.y, baseIndex, value*weightLower);       
        tempBuffer.add(pos.x, pos.y, baseIndex+1, value*weightUpper);
    }
}
   

float OutlierDetectorZirr1::calculateWeight(Point2 const& posFloat, float value) const {
    auto pos = discretePosition(posFloat);
    assert(pos.y < height && pos.x < width);

    if (value > maxValue) {
        return 1;
    }

    if (value < minValue) {
        return 0;
    }

    int index = std::min(std::max(0.f, std::floor(std::log(value) / std::log(b))), nbBuffers-1.f);

    float result = 0;
    float reliability;

    for (int curr=0; curr <= index; ++curr) {

        float currScale = spp*oneOverK/powersOfb[curr];

        /* sample counting-based reliability estimation */

        // reliability in 3x3 pixel block (see robustness)
        float globalReliability = sampleReliability(pos, curr, 1, currScale);
        // reliability of curent pixel
        float localReliability = sampleReliability(pos, curr, 0, currScale);

        reliability = globalReliability - oneOverK;
        // check if above minimum sampling threshold
        if (reliability >= 0.)
            // then use per-pixel reliability
            reliability = localReliability - oneOverK;

        /* color-based reliability estimation */

        float colorReliability = result * currScale;

        // a minimum image brightness that we always consider reliable
        colorReliability = std::max(colorReliability, 0.05f * currScale);

        // if not interested in exact expected value estimation, can usually accept a bit
        // more variance relative to the image brightness we already have
        float optimizeForError = std::max(.0f, std::min(1.f, oneOverK));
        // allow up to ~<cascadeBase> more energy in one sample to lessen bias in some cases
        colorReliability *= (0.4f + 0.6f*b) * (1-optimizeForError) + optimizeForError; // needed?
        
        // reliability = (reliability + colorReliability) * .5f;
        reliability = std::min(reliability, colorReliability);

        reliability = math::clamp(reliability, 0.f, 1.f);

        result += reliability * buffer.get(pos.x, pos.y, curr);
    }

    sumReliabilities += reliability;
    ++countReliabilities;

    if (reliability < threshold) {
        return 1;
    } else {
        return 0;
    }

}


// average of <r>-radius pixel block in <layer> at <coord> (only using r=0 and r=1)
float OutlierDetectorZirr1::sampleLayer(int layer, Point2i pos, const int r, float scale) const {
	float val = 0;
	for (int y = -r; y <= r; ++y) {
		for (int x = -r; x <= r; ++x) {
            x += pos.x;
            y += pos.y;
            if (x >= 0 && x < width && y >= 0 && y < height) {
                float c = buffer.get(x, y, layer);
                c *= scale;
                val += c;	
            }		
		}
	}
	return val / float((2*r+1)*(2*r+1));
}

// sum of reliabilities in <curr> layer, <prev> layer and <next> layer
float OutlierDetectorZirr1::sampleReliability(Point2i coord, int curr, const int r, float currScale) const {
	float rel = sampleLayer(curr, coord, r, currScale);
	if (curr != 0)
		// scale by N/kappa / b^i_<pref>
		rel += sampleLayer(curr-1, coord, r, currScale * b); // Changed the / and * from place, seems to work?
	if (curr < nbBuffers-1)
		// scale by N/kappa / b^i_<next>
		rel += sampleLayer(curr+1, coord, r, currScale / b);
	// reliability is simply the luminance of the brightness-normalized layer pixels
	return rel;
}


void OutlierDetectorZirr1::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
    update(newSpp);
}

void OutlierDetectorZirr1::update(int newSpp) {
    buffer.add(tempBuffer);
    tempBuffer.reset();
    spp += newSpp;

    std::cout << "Avg reliablility: " << sumReliabilities/countReliabilities << std::endl;

    sumReliabilities = 0;
    countReliabilities = 0;
}



MTS_NAMESPACE_END