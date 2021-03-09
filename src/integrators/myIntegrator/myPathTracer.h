
#include <mitsuba/core/properties.h>
#include <mitsuba/render/integrator.h>
#include <mitsuba/core/bitmap.h>

#include <string>

#include "../pssmlt/pssmlt.h"
#include "my_pathSeed.h"

#include "outlierDetectors/bitterli.h"
#include "outlierDetectors/zirr1.h"


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

    /// Unserialize an integrator
    MyPathTracer(Stream *stream, InstanceManager *manager);

    /**
     * \brief Perform the main rendering task
     *
     * The work is automatically parallelized to multiple cores and
     * remote machines. The default implementation uniformly generates
     * samples on the sensor aperture and image plane as specified by
     * the used sampler. The average of the estimated radiance along the
     * associated rays in a pixel region is then taken as an approximation
     * of that pixel's radiance value. For adaptive strategies, have a look at
     * the \c adaptive plugin, which is an extension of this class.
     */
    bool render(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID);

    /**
     * This can be called asynchronously to cancel a running render job.
     * In this case, <tt>render()</tt> will quit with a return value of
     * <tt>false</tt>.
     */
    void cancel();

    /**
     * This method does the main work of <tt>render()</tt> and
     * runs in parallel for a series of image blocks, which are
     * being processed at a time.
     *
     * \param scene
     *    Pointer to the underlying scene
     * \param sensor
     *    Pointer to the sensor used to render the image
     * \param sampler
     *    Pointer to the sampler used to render the image
     * \param block
     *    Pointer to the image block to be filled
     * \param points
     *    Specifies the traversal order, i.e. using a space-filling
     *    curve. To limit the size of the array, it is currently assumed
     *    that the block size is smaller than 256x256
     * \param stop
     *    Reference to a boolean, which will be set to true when
     *    the user has requested that the program be stopped
     */
    virtual void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points);

    
    size_t computeMltBudget() const;

    /**
     * <tt>NetworkedObject</tt> implementation:
     * When a parallel rendering process starts, the integrator is
     * given the opportunity to attach globally shared resources to
     * the process. This is useful for distributing heavy data
     * structures (e.g. photon maps) without having to re-transmit
     * them every time an image is rendered.
     */
    virtual void bindUsedResources(ParallelProcess *proc) const;

    /**
     * <tt>NetworkedObject</tt> implementation:
     * Called once just before this integrator instance is asked
     * to process an image block. In comparison to <tt>preprocess()</tt>
     * this will be executed on _every_ instance of this class, which is
     * useful for connecting to globally shared resources (photon maps,
     * irradiance caches, ..) after having been unserialized on a
     * remote machine. A list of resources bound to the associated
     * parallel process is given as a parameter.
     */
    virtual void wakeup(ConfigurableObject *parent,
        std::map<std::string, SerializableObject *> &params);

    /// Serialize this integrator to a binary data stream
    void serialize(Stream *stream, InstanceManager *manager) const;

    MTS_DECLARE_CLASS()

private:
    /// Draw nChains samples from allSeeds proportional to their luminance. Samples can be chosen multiple times.
    inline std::vector<PositionedPathSeed> drawSeeds(std::vector<PositionedPathSeed>& allSeeds, size_t nChains, Sampler* sampler) {
        
        DiscreteDistribution seedDistribution(allSeeds.size());
        for (auto& seed : allSeeds) {
            seedDistribution.append(seed.luminance);        
        }

        seedDistribution.normalize();

        // Log(EInfo, seedDistribution.toString().c_str());

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

        // std::vector<PositionedPathSeed> seeds;
        // seeds.reserve(nChains);
        // for (size_t i=0; i<nChains; i++) {
        //     int index = std::floor(sampler->next1D() * allSeeds.size());
        //     auto seed = allSeeds[index];
        //     seed.pdf = 1.f/nChains;
        //     seeds.push_back(seed); 
        // }
        // return seeds;
    }

private:
    /// Used to temporarily cache a parallel process while it is in operation
    ref<ParallelProcess> m_process;
    PSSMLTConfiguration m_config;
    std::vector<PositionedPathSeed> pathSeeds;
    size_t samplesPerPixel, samplesTotal;
    Vector2 invSize;
    // ref<OutlierDetectorZirr1> detector;
    ref<OutlierDetectorBitterly> detector;
    int iteration, iterations;
    RunningAverage<Float> unweightedAvg, weightedAvg;
    ref<Mutex> seedMutex;
    ref<Bitmap> pathResult;
    bool noMlt;
    Float outlierDetectorThreshold;
    int intermediatePeriod;
    std::string intermediatePath;
};

MTS_NAMESPACE_END

#endif
