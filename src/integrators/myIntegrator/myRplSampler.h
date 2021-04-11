

#ifndef MYRPLSAMPLER
#define MYRPLSAMPLER

#include <mitsuba/bidir/rsampler.h>

MTS_NAMESPACE_BEGIN

class MyRplSampler : public ReplayableSampler {

public:
    void reSeed(uint64_t seed) {
        m_initial = new Random(seed);
        m_random = new Random(m_initial);
        // m_random->set(m_initial);
        m_sampleIndex = 0;
        // m_sampleCount = 0;
    }

};

MTS_NAMESPACE_END

#endif
