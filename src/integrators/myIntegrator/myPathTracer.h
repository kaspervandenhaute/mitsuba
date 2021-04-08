
#include <mitsuba/core/properties.h>
#include <mitsuba/render/integrator.h>
#include <mitsuba/core/bitmap.h>
#include "utils/writeBitmap.h"

#include <mitsuba/core/statistics.h>

#include <string>
#include <iostream>
#include <fstream>

#include "my_pathSeed.h"
#include "xxHash/xxhash.h"

#include "myPssmltSampler.h"
#include "myRplSampler.h"

#include "outlierDetectors/outlierDetector.h"
#include "myPSSMLTconfig.h"


#ifndef MY_PATH_TRACER
#define MY_PATH_TRACER

MTS_NAMESPACE_BEGIN

struct MltStats;

template<typename T>
class Average {

public: 
    Average() : avg(0), count(0) {}

    inline T get() const {
        return avg;
    }

    inline void put(T val) {
        put(val, 1);
    }

    inline void put(T val, int n) {
        auto prev_count = count;
        count += n;
        avg = avg * ((float) (prev_count)/count) + val/count;
    }

    void reset() {
        avg = 0;
        count = 0;
    }

private:
    T avg;
    size_t count;
};

// template<typename T>
// class RunningAverage {

// public: 
//     RunningAverage(int nValues) : avg(0), count(0), n(nValues), current(0) {
//         numbers = new T[](nValues);
//     }

//     inline T get() const {
//         int total = 0;
//         for (int i=0; i<n; ++i) {
//             total += numbers[i];
//         }
//         return total / n;
//     }

//     inline void put(T val) {

//     }

//     inline void put(T val, int n) {

//     }

//     void reset() {

//     }

// private:
//     T avg;
//     size_t count;
//     int n;
//     T* numbers;
// };


class MyPathTracer : public Integrator {
public:

    /// Create a integrator
    MyPathTracer(const Properties &props);

    bool render(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID);


    virtual void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points);


    MyPathTracer(Stream *stream, InstanceManager *manager)
    : Integrator(stream, manager) { 
            seedMutex = new Mutex();

    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        Integrator::serialize(stream, manager);
    }

    void cancel() {
        if (m_process)
            Scheduler::getInstance()->cancel(m_process);
    }
    
    virtual void bindUsedResources(ParallelProcess *proc) const {};

    virtual void wakeup(ConfigurableObject *parent,
        std::map<std::string, SerializableObject *> &params) {};

    MTS_DECLARE_CLASS()

private:
    /// Draw nChains samples from allSeeds proportional to their luminance. Samples can be chosen multiple times.
    inline std::vector<PositionedPathSeed> drawSeeds(std::vector<PositionedPathSeed>& allSeeds, size_t nChains, Sampler* sampler) const {
        
        DiscreteDistribution seedDistribution(allSeeds.size());
        for (auto& seed : allSeeds) {
            seedDistribution.append(seed.luminance);        
        }

        seedDistribution.normalize();

        std::vector<PositionedPathSeed> seeds;
        seeds.reserve(nChains);
        for (size_t i=0; i<nChains; ++i) {
            Float pdf;
            auto index = seedDistribution.sample(sampler->next1D(), pdf);
            auto seed = allSeeds[index];
            seed.pdf = pdf;
            seeds.push_back(seed);                    
        }
        return seeds;
    }

    inline size_t computeMltBudget() const {
    Float r = weightedAvg.get() / unweightedAvg.get();
    // When all samples are outliers r will be close to 1
    if (r >= 0.99 && r <= 1) {
        return samplesTotal;
    } else if (r > 1) {
        assert(r <= 1);
    }
    return samplesTotal * r / (1-r);
}

    inline uint64_t createSeed(Point2i const& pos) const {
        int buffer[3] = {pos.x, pos.y, iteration};
        return (uint64_t) XXH3_64bits_withSeed((void*) buffer, sizeof(int)*3, 0);
    }

    void writeAvos(std::string const& path, std::string const& tag= "") {

        auto result = mltResult->clone();
        result->accumulate(pathResult->getBitmap(), Point2i(pathResult->getBorderSize()), Point2i(0.f), result->getSize());
        result->scale(1.f/iterations);
        BitmapWriter::writeBitmap(result, BitmapWriter::EHDR, path + "total" + tag + ".exr");

        mltResult->scale(1.f/iterations);
        BitmapWriter::writeBitmap(mltResult, BitmapWriter::EHDR, path + "mlt" + tag + ".exr");

        result->clear();
        result->accumulate(seedsResult->getBitmap(), Point2i(pathResult->getBorderSize()), Point2i(0.f), result->getSize());
        result->scale(1.f/iterations);
        BitmapWriter::writeBitmap(result, BitmapWriter::EHDR, path + "seeds" + tag + ".exr");

        result->clear();
        result->accumulate(outliersResult->getBitmap(), Point2i(pathResult->getBorderSize()), Point2i(0.f), result->getSize());
        result->scale(1.f/iterations);
        BitmapWriter::writeBitmap(result, BitmapWriter::EHDR, path + "outliers" + tag + ".exr");

        result->clear();
        result->accumulate(pathResult->getBitmap(), Point2i(pathResult->getBorderSize()), Point2i(0.f), result->getSize());
        result->scale(1.f/iterations);
        BitmapWriter::writeBitmap(result, BitmapWriter::EHDR, path + "path" + tag + ".exr");
    }

    void writeTotal(std::string const& path) {
        auto result = createResult();
        BitmapWriter::writeBitmap(result, BitmapWriter::EHDR, path);
    }

    ref<Bitmap> createResult() {
        ref<Bitmap> result = mltResult->clone();
        result->scale(1.f/samplesPerPixel); //TODO: Why?
        result->accumulate(pathResult->getBitmap(), Point2i(pathResult->getBorderSize()), Point2i(0.f), result->getSize());
        result->scale(1.f/iteration);
        return result;
    }

    void clearResults() {
        mltResult->clear();
        pathResult->clear();
        outliersResult->clear();
        seedsResult->clear();
    }

    void updateOutlierSeedsStats(std::vector<PositionedPathSeed> const& outliers, std::vector<PositionedPathSeed> const& seeds, StatsCounter& outlierCounter, StatsCounter& seedsCounter) {
        // Write all outliers and seeds for later analysis
        for (auto& o : pathSeeds) {
            outliersResult->put(Point2(o.position.x * cropSize.x, o.position.y * cropSize.y), o.spec, 1);
        }
        for (auto& o : seeds) {
            seedsResult->put(Point2(o.position.x * cropSize.x, o.position.y * cropSize.y), o.spec, 1);
        }
        outlierCounter += pathSeeds.size();
        seedsCounter += seeds.size();
    }

    void initialiseSamplers(int& mltSamplerResID, int& rplSamplerResID) {
        // mlt samplers
        ref<MyPSSMLTSampler> mltSampler = new MyPSSMLTSampler(m_config.mutationSizeLow, m_config.mutationSizeHigh);
        std::vector<SerializableObject *> mltSamplers(sched->getCoreCount());
        for (size_t i=0; i<mltSamplers.size(); ++i) {
            ref<Sampler> clonedSampler = mltSampler->clone();
            clonedSampler->incRef();
            mltSamplers[i] = clonedSampler.get();
        }
        mltSamplerResID = sched->registerMultiResource(mltSamplers);
        for (size_t i=0; i<sched->getCoreCount(); ++i)
            mltSamplers[i]->decRef();
    
        // path tracer samplers
        ref<MyRplSampler> rplSampler = new MyRplSampler();
        std::vector<SerializableObject*> rplSamplers(sched->getCoreCount());
        for (size_t i=0; i<rplSamplers.size(); ++i) {
            ref<Sampler> clonedSampler = rplSampler->clone();
            clonedSampler->incRef();
            rplSamplers[i] = clonedSampler.get();
        }
        rplSamplerResID = sched->registerMultiResource(rplSamplers);
        for (size_t i=0; i<sched->getCoreCount(); ++i)
            rplSamplers[i]->decRef();
    }

    void pathTracing(Scene *scene, RenderQueue *queue, const RenderJob *job, 
    int sceneResID, int sensorResID, int samplerResID, int rplSamplerResID, int integratorResID);

    void initDetector(Scene *scene, RenderQueue *queue, const RenderJob *job, 
    int sceneResID, int sensorResID, int samplerResID, int rplSamplerResID, int integratorResID);

    void writeStatisticsToFile(int nOutliers, int nSeeds, MltStats mltStats) const;

    void init();

    void renderSetup(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID);

    bool myRender(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID);


private:
    /// Used to temporarily cache a parallel process while it is in operation
    ref<ParallelProcess> m_process;
    MYPSSMLTConfiguration m_config;
    std::vector<PositionedPathSeed> pathSeeds;
    size_t samplesPerPixel, samplesTotal;
    Vector2 invSize;
    ref<OutlierDetector> detector;
    int iteration, iterations;
    Average<Float> unweightedAvg, weightedAvg;
    ref<Mutex> seedMutex;
    ref<ImageBlock> pathResult;
    ref<Bitmap> mltResult;
    ref<ImageBlock> outliersResult;
    ref<ImageBlock> seedsResult;
    Float outlierDetectorThreshold;
    int intermediatePeriod;
    std::string intermediatePath;
    ref<Random> random;
    uint64_t cost = 0;

    ref<Scheduler> sched;
    ref<Sensor> sensor;
    ref<Film> film;
    Sampler *sampler;
    size_t nCores;
    Vector2i cropSize;

    Properties props;
    int nPoints, nSubPoints;
    std::string testProperty;
    bool exponential;
    float minValue, maxValue;
};

MTS_NAMESPACE_END

#endif
