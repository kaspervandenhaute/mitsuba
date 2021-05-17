
#include "meanDetector.h"

MTS_NAMESPACE_BEGIN

void MeanOutlierDetector::contribute(Point2 const& posFloat, float value) {
    int index = get_index(posFloat);

    ++counts_temp[index];

    // initialises the sum as the median of the first iteration.
    // assumes all the samples of a pixel are done one after the other.

    // TODO: doesn't work multi threaded
    if (current_spp == 0) {  // initialisation
        if (index == last_index) {
            init_samples.push_back(value);
        } else if (last_index != -1) {
            std::sort(init_samples.begin(), init_samples.end());
            sums_temp[last_index] = init_samples[init_samples.size()*9/10];
            init_samples.clear();
            init_samples.push_back(value);
        }
        last_index = index;
    } else {
        auto weight = calculateWeightIndex(index, value);
        // Only add non outliers otherwise the mean will break down
        if (weight == 0.f) {
            sums_temp[index] += value;           
        }
    }
}


float MeanOutlierDetector::calculateWeight(Point2 const& posFloat, float value) const  {
    int index = get_index(posFloat);

    return calculateWeightIndex(index, value);
}


float MeanOutlierDetector::calculateWeightIndex(const int index, const float value) const  {

    auto current_avg = sums[index]/ counts[index];
    auto avg_with_new = (current_avg * total_spp + value) / (total_spp + 1);

    // std::cout << "current: " << current_avg << "  new: " << avg_with_new << '\n';

    if (avg_with_new > current_avg * threshold) {
        return 1.f;
    }
    return 0.f;
}


void MeanOutlierDetector::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
    update(newSpp);
}


void MeanOutlierDetector::update(int newSpp) {
    std::transform(sums_temp.begin(), sums_temp.end(), sums.begin(), sums.begin(), std::plus<float>());
    std::transform(counts_temp.begin(), counts_temp.end(), counts.begin(), counts.begin(), std::plus<uint32_t>());

    std::fill(sums_temp.begin(), sums_temp.end(), 0.f);
    std::fill(counts_temp.begin(), counts_temp.end(), 0);

    current_spp += newSpp;
    //assert(counts[0] == current_spp);
}


MTS_NAMESPACE_END

