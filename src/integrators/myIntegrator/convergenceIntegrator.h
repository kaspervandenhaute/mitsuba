#include <mitsuba/core/properties.h>
#include <mitsuba/render/integrator.h>

#ifndef CONVERGENDEINTEGRATOR
#define CONVERGENDEINTEGRATOR

MTS_NAMESPACE_BEGIN

class ConvergenceIntegrator : public Integrator {
public:

    /// Create a integrator
    ConvergenceIntegrator(const Properties &props);

    /// Unserialize an integrator
    ConvergenceIntegrator(Stream *stream, InstanceManager *manager);

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
    Properties props;

};
MTS_NAMESPACE_END

#endif