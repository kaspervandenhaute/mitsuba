

#include "MutatablePssmltSampler.h"

MTS_NAMESPACE_BEGIN

void MutatablePSSMLTSampler::setPrimarySample(int index, Float value) {
        assert(value >= 0 && value <= 1);
        assert(index <= m_u.size());
        if (m_u.size() > index) {
            m_u[index].value = value;
        } else {
            m_u.emplace_back(value);
        }
    }

MTS_IMPLEMENT_CLASS(MutatablePSSMLTSampler, false, PSSMLTSampler);

MTS_NAMESPACE_END

