
#ifndef MITSUBA_MUTATABLE_PSSMLT_SAMPLER_H
#define MITSUBA_MUTATABLE_PSSMLT_SAMPLER_H

#include "../pssmlt/pssmlt_sampler.h"

MTS_NAMESPACE_BEGIN


class MutatablePSSMLTSampler : public PSSMLTSampler {
public:

    using PSSMLTSampler::PSSMLTSampler;

    /// Carefull with this!!!
    void setPrimarySample(int index, Float value);

    MTS_DECLARE_CLASS()
};

MTS_NAMESPACE_END

#endif