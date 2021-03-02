
#ifndef MITSUBA_OUTLIERDETECTOR_H
#define MITSUBA_OUTLIERDETECTOR_H

#include <array>
#include <vector>
#include <cassert>

#include <mitsuba/mitsuba.h>

#include "../my_pathSeed.h"


MTS_NAMESPACE_BEGIN

struct RatioAndIndex {
    float ratio;
    int index;
};

template <typename T>
struct Bitmap3d {
    T* data;
    size_t width, height, depth;

    Bitmap3d(size_t width, size_t height, size_t depth) : 
        width(width), height(height), depth(depth) {
        assert(width > 0 && height > 0 && depth > 0);
        data = new T[size()]();
    }

    inline T get(size_t x, size_t y, size_t z) {
        assert(x < width && y < height && z < depth);
        return data[x + width*y + width*height*z];
    }

    inline void set(size_t x, size_t y, size_t z, T value) {
        assert(x < width && y < height && z < depth);
        data[x + width*y + width*height*z] = value;
    }

    inline void add(size_t x, size_t y, size_t z, T value) {
        // std::cout << z << "  " << depth << std::endl;
        assert(x < width);
        assert(y < height);
        assert(z < depth);
        data[x + width*y + width*height*z] += value;
    }

    inline void add(Bitmap3d const& other) {
        assert(width == other.width && height == other.height && depth == other.depth);
        for (size_t i=0; i<size(); ++i) {
            data[i] += other.data[i];
        }
    }

    inline void reset() {
        for (size_t i=0; i<size(); ++i) {
            data[i] = 0.f;
        }
    }

    inline size_t size() {
        return width*height*depth;
    }

    ~Bitmap3d() {
        delete data;
    }
};

class OutlierDetectorBitterly {

public:
    OutlierDetectorBitterly(int width, int height, int nbBuffers, float alfa, float beta, float maxValue);

    void contribute(Point2i const& pos, float value);
    float calculateWeight(Point2i const& pos, float value, int spp);
    void update(std::vector<PositionedPathSeed> const& seeds, size_t nChains);

private:
    const int width, height;
    const int nbBuffers;
    const float alfaInv, beta;
    float maxValue, minValue;
    Bitmap3d<float> buffer, tempBuffer;
    float minThreshold = 3;


    float calcualateOccurencies(Point2i const& pos, float value);
    float calculateThreshold(Point2i const& pos, int spp);
    RatioAndIndex calculateRatioAndIndex(float value);
    void setAdditionalThreshold(std::vector<PositionedPathSeed> const& seeds, size_t nChains);

};

MTS_NAMESPACE_END


#endif //RAY_TRACER_OUTLIERDETECTOR_H
