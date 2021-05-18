
#include "meanDetector.h"

#include "../utils/writeBitmap.h"
#include "../utils/thesisLocation.h"

MTS_NAMESPACE_BEGIN

void MeanOutlierDetector::contribute(Point2 const& posFloat, float value) {
    auto pos = discrete_pos(posFloat);
    int index = get_index(pos);

    auto weight = calculateWeightDiscrete(pos, value);
    // Only add non outliers otherwise the mean will break down
    if (weight == 0.f) {
        sums_temp[index] += value;           
    }
    
}


void MeanOutlierDetector::init(Point2i const& pos, const float value) {
    current_mean[get_index(pos)] = value;
}


void MeanOutlierDetector::contributeMlt(Point2 const& posFloat, float value) {
    auto pos = discrete_pos(posFloat);
    auto index = get_index(pos);

    mlt_sums_temp[index] += value;
    ++mlt_samples;
}


float MeanOutlierDetector::calculateWeight(Point2 const& posFloat, float value) const {
    auto pos = discrete_pos(posFloat);

    return calculateWeightDiscrete(pos, value);
}


float MeanOutlierDetector::calculateWeightDiscrete(Point2i const& pos, const float value) const {

    float mean = neighbors_mean(pos);

    auto avg_with_new = (mean * total_spp + value) / (total_spp + 1);

    // std::cout << "current: " << current_avg << "  new: " << avg_with_new << '\n';

    if (avg_with_new > mean * threshold) {
        return 1.f;
    }
    return 0.f;
}


void MeanOutlierDetector::update(std::vector<PositionedPathSeed> const& seeds, size_t nChains, int newSpp) {
    update(newSpp);
}


void MeanOutlierDetector::update(int newSpp) {

    float inv_spp = 1.f / newSpp;
    float inv_mlt_samples = 1.f / (mlt_samples*newSpp); //TODO: doesn't work with mlt_samples == 0
    auto combine_path_mlt = [inv_spp, inv_mlt_samples](const float a, const float b) {
        return (a*inv_spp + b*inv_mlt_samples);
    };

    float inv_next_iteration = 1.f / (iteration+1);

    if (iteration != 0) {
        std::transform(sums_temp.begin(), sums_temp.end(), mlt_sums_temp.begin(), sums_temp.begin(), combine_path_mlt);
        std::transform(sums_temp.begin(), sums_temp.end(), current_mean.begin(), current_mean.begin(), 
            [this, inv_next_iteration](const float a, const float b){
                return (a + b*iteration) * inv_next_iteration;
            });
        

        std::fill(sums_temp.begin(), sums_temp.end(), 0.f);
        std::fill(mlt_sums_temp.begin(), mlt_sums_temp.end(), 0.f);
        mlt_samples = 0;
    }

    auto bitmap = new Bitmap(Bitmap::ELuminance, Bitmap::EFloat32, Vector2i(width, height), 1, reinterpret_cast<uint8_t*>(current_mean.data()));
    BitmapWriter::writeBitmap(bitmap, BitmapWriter::EHDR, THESISLOCATION + "prentjes/test/mean/" + std::to_string(iteration) + ".exr");

    current_spp += newSpp;
    ++iteration;
}

float MeanOutlierDetector::neighbors_mean(Point2i const& pos, const int r) const {
    float avg = 0;
    for (int x=std::max(pos.x-r, 0); x<=std::min(pos.x+r, width); ++x) {
        for (int y=std::max(pos.y-r, 0); y<=std::min(pos.y+r, height); ++y) {
            avg += current_mean[get_index(x, y)];
        }
    }
    return avg / float((2*r+1)*(2*r+1)); // not quite correct on edges and corners
}


MTS_NAMESPACE_END

