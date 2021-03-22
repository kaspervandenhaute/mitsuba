/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "myPSSMLTSampler.h"

MTS_NAMESPACE_BEGIN

MyPSSMLTSampler::MyPSSMLTSampler(Float mutationSizeLow, Float mutationSizeHigh) : Sampler(Properties()) {
    m_random = new Random();
    m_s1 = mutationSizeLow;
    m_s2 = mutationSizeHigh;
    configure();
}

MyPSSMLTSampler::MyPSSMLTSampler(MyPSSMLTSampler *sampler) : Sampler(Properties()),
    m_random(sampler->m_random) {
    m_s1 = sampler->m_s1;
    m_s2 = sampler->m_s2;
    configure();
}

MyPSSMLTSampler::MyPSSMLTSampler(Stream *stream, InstanceManager *manager)
    : Sampler(stream, manager) {
    m_random = static_cast<Random *>(manager->getInstance(stream));
    m_s1 = stream->readFloat();
    m_s2 = stream->readFloat();
    configure();
}

void MyPSSMLTSampler::serialize(Stream *stream, InstanceManager *manager) const {
    Sampler::serialize(stream, manager);
    manager->serialize(stream, m_random.get());
    stream->writeFloat(m_s1);
    stream->writeFloat(m_s2);
}

void MyPSSMLTSampler::configure() {
    m_logRatio = -math::fastlog(m_s2/m_s1);
    m_time = 0;
    m_largeStepTime = 0;
    m_largeStep = false;
    m_sampleIndex = 0;
    m_sampleCount = 0;
}

MyPSSMLTSampler::~MyPSSMLTSampler() { }

void MyPSSMLTSampler::setPrimarySample(int index, Float value) {
    assert(value >= 0 && value <= 1);
    assert(index <= m_u.size());
    if (m_u.size() > index) {
        m_u[index].value = value;
    } else {
        m_u.emplace_back(value);
    }
    
}

void MyPSSMLTSampler::accept() {
    if (m_largeStep)
        m_largeStepTime = m_time;
    m_time++;
    m_backup.clear();
    m_sampleIndex = 0;
}

void MyPSSMLTSampler::reset() {
    m_time = m_sampleIndex = m_largeStepTime = 0;
    m_u.clear();
}

void MyPSSMLTSampler::reject() {
    for (size_t i=0; i<m_backup.size(); ++i)
        m_u[m_backup[i].first] = m_backup[i].second;
    m_backup.clear();
    m_sampleIndex = 0;
}

Float MyPSSMLTSampler::primarySample(size_t i) {

    while (i >= m_u.size())
        m_u.push_back(SampleStruct(m_random->nextFloat()));

    if (m_u[i].modify < m_time) {
        if (m_largeStep) {
            m_backup.push_back(std::pair<size_t, SampleStruct>(i, m_u[i]));
            m_u[i].modify = m_time;
            m_u[i].value = m_random->nextFloat();
        } else {
            if (m_u[i].modify < m_largeStepTime) {
                m_u[i].modify = m_largeStepTime;
                m_u[i].value = m_random->nextFloat();
            }

            while (m_u[i].modify + 1 < m_time) {
                m_u[i].value = mutate(m_u[i].value);
                m_u[i].modify++;
            }

            m_backup.push_back(std::pair<size_t, SampleStruct>(i, m_u[i]));

            m_u[i].value = mutate(m_u[i].value);
            m_u[i].modify++;
        }
    }

    return m_u[i].value;
}

ref<Sampler> MyPSSMLTSampler::clone() {
    ref<MyPSSMLTSampler> sampler = new MyPSSMLTSampler(this);
    sampler->m_sampleCount = m_sampleCount;
    sampler->m_sampleIndex = m_sampleIndex;
    sampler->m_random = new Random(m_random);
    return sampler.get();
}

Float MyPSSMLTSampler::next1D() {
    return primarySample(m_sampleIndex++);
}

Point2 MyPSSMLTSampler::next2D() {
    /// Enforce a specific order of evaluation
    Float value1 = primarySample(m_sampleIndex++);
    Float value2 = primarySample(m_sampleIndex++);
    return Point2(value1, value2);
}

std::string MyPSSMLTSampler::toString() const {
    std::ostringstream oss;
    oss << "MyPSSMLTSampler[" << endl
        << "  sampleCount = " << m_sampleCount << endl
        << "]";
    return oss.str();
}

MTS_IMPLEMENT_CLASS_S(MyPSSMLTSampler, false, Sampler)
MTS_NAMESPACE_END

