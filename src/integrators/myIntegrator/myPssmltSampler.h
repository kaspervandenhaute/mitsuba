#include <mitsuba/mitsuba.h>
#include <mitsuba/render/sampler.h>
#include <mitsuba/core/random.h>
#include <mitsuba/core/math.h>

#ifndef MYPSSMLTSAMPLER
#define MYPSSMLTSAMPLER

MTS_NAMESPACE_BEGIN

/**
 * Sampler implementation as described in
 * 'A Simple and Robust Mutation Strategy for the
 * Metropolis Light Transport Algorithm' by Kelemen et al.
 */
class MyPSSMLTSampler : public Sampler {
public:
    // Construct a new MLT sampler
    MyPSSMLTSampler(Float mutationSizeLow, Float mutationSizeHigh);

    /**
     * \brief Construct a new sampler, which operates on the
     * same random number generator as \a sampler.
     */
    MyPSSMLTSampler(MyPSSMLTSampler *sampler);

    /// Unserialize from a binary data stream
    MyPSSMLTSampler(Stream *stream, InstanceManager *manager);

    /// Set up the internal state
    void configure();

    /// Serialize to a binary data stream
    void serialize(Stream *stream, InstanceManager *manager) const;

    /// Set whether the current step should be large
    inline void setLargeStep(bool value) { m_largeStep = value; }

    /// Check if the current step is a large step
    inline bool isLargeStep() const { return m_largeStep; }

    /// Retrieve the next component value from the current sample
    virtual Float next1D();

    /// Retrieve the next two component values from the current sample
    virtual Point2 next2D();

    /// Return a string description
    virtual std::string toString() const;

    /// 1D mutation routine
    inline Float mutate(Float value) {
        #if KELEMEN_STYLE_MUTATIONS == 1
            Float sample = m_random->nextFloat();
            bool add;

            if (sample < 0.5f) {
                add = true;
                sample *= 2.0f;
            } else {
                add = false;
                sample = 2.0f * (sample - 0.5f);
            }

            Float dv = m_s2 * math::fastexp(sample * m_logRatio);
            if (add) {
                value += dv;
                if (value > 1)
                    value -= 1;
            } else {
                value -= dv;
                if (value < 0)
                    value += 1;
            }
        #else
            Float tmp1 = std::sqrt(-2 * std::log(1-m_random->nextFloat()));
            Float dv = tmp1 * std::cos(2*M_PI*m_random->nextFloat());
            value = math::modulo(value + 1e-2f * dv, 1.0f);
        #endif

        return value;
    }

    /// Return a primary sample
    Float primarySample(size_t i);

    /// Carefull with this!!!
    void setPrimarySample(int index, Float value);

    /// Reset (& start with a large mutation)
    void reset();

    /// Accept a mutation
    void accept();

    /// Reject a mutation
    void reject();

    /// Replace the underlying random number generator
    inline void setRandom(Random *random) { m_random = random; }

    /// Return the underlying random number generator
    inline Random *getRandom() { return m_random; }

    /* The following functions do nothing in this implementation */
    virtual void advance() { }
    virtual void generate(const Point2i &pos) { }

    /* The following functions are unsupported by this implementation */
    void request1DArray(size_t size) { Log(EError, "request1DArray(): Unsupported!"); }
    void request2DArray(size_t size) { Log(EError, "request2DArray(): Unsupported!"); }
    void setSampleIndex(size_t sampleIndex) { Log(EError, "setSampleIndex(): Unsupported!"); }
    ref<Sampler> clone();

    MTS_DECLARE_CLASS()
protected:
    /// Virtual destructor
    virtual ~MyPSSMLTSampler();
protected:
    struct SampleStruct {
        Float value;
        size_t modify;

        inline SampleStruct(Float value) : value(value), modify(0) { }
    };

    ref<Random> m_random;
    Float m_s1, m_s2, m_logRatio;
    bool m_largeStep;
    std::vector<std::pair<size_t, SampleStruct> > m_backup;
    std::vector<SampleStruct> m_u;
    size_t m_time, m_largeStepTime;
    Float m_probLargeStep;
};

MTS_NAMESPACE_END


#endif

