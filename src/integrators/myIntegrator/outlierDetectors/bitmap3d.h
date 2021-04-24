#ifndef MITSUBA_BITMAP3D_H
#define MITSUBA_BITMAP3D_H

#include <mitsuba/mitsuba.h>

#include "../utils/writeToBinary.h"
#include "../utils/thesisLocation.h"


MTS_NAMESPACE_BEGIN

struct RatioAndIndex {
    float ratio;
    int index;
};

template <typename T>
struct Bitmap3d {
    
    size_t width, height, depth;
    T* data;

    Bitmap3d(size_t width, size_t height, size_t depth) : 
        width(width), height(height), depth(depth) {
        assert(width > 0 && height > 0 && depth > 0);
        data = new T[size()]();
    }

    Bitmap3d(size_t width, size_t height, size_t depth, T* data) : 
        width(width), height(height), depth(depth), data(data) {
        assert(width > 0 && height > 0 && depth > 0);
       
    }

    inline T get(size_t x, size_t y, size_t z) const {
        assert(x < width);
        assert(y < height);
        assert(z < depth);
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

    inline size_t size() const {
        return width*height*depth;
    }

    ~Bitmap3d() {
        delete[] data;
    }

    void writeToBinary(std::string path) {
        writeToBinaryFile(data, width*height*depth, path);
    }
};

MTS_NAMESPACE_END

#endif