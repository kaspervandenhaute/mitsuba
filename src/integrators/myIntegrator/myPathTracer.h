
#include <mitsuba/core/properties.h>
#include <mitsuba/render/integrator.h>
#include <mitsuba/core/bitmap.h>

#include <string>

#include "my_pathSeed.h"
#include "xxHash/xxhash.h"

#include "outlierDetectors/outlierDetector.h"
#include "myPSSMLTconfig.h"


#ifndef MY_PATH_TRACER
#define MY_PATH_TRACER

MTS_NAMESPACE_BEGIN

template<typename T>
class RunningAverage {

public: 
    RunningAverage() : avg(0), count(0) {}

    T get() const {
        return avg;
    }

    void put(T val) {
        ++count;
        avg = avg * ((float) (count-1)/count) + val/count;
    }

private:
    T avg;
    size_t count;
};


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
    
    size_t computeMltBudget() const;

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

    inline uint64_t createSeed(Point2i const& pos) const {
        int buffer[3] = {pos.x, pos.y, iteration};
        return (uint64_t) XXH3_64bits_withSeed((void*) buffer, sizeof(int)*3, 0);
    }

    void init();

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
    RunningAverage<Float> unweightedAvg, weightedAvg;
    ref<Mutex> seedMutex;
    ref<ImageBlock> pathResult;
    bool noMlt;
    Float outlierDetectorThreshold;
    int intermediatePeriod;
    std::string intermediatePath;
    ref<Random> random;
    uint64_t cost = 0;


    Properties props;
    int nPoints, nSubPoints;
    std::string testProperty;
    bool exponential;
    float minValue, maxValue;
};

MTS_NAMESPACE_END

#endif
