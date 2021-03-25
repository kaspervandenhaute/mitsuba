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

#include <mitsuba/bidir/util.h>
#include <mitsuba/bidir/path.h>
#include "my_pssmlt_proc.h"
#include "myPssmltSampler.h"
#include "my_pathSeed.h"
#include "myPathTracer.h"
#include "myRplSampler.h"

#include <string>


MTS_NAMESPACE_BEGIN

/* ==================================================================== */
/*                         Worker implementation                        */
/* ==================================================================== */


StatsCounter smallStepRatio("Primary sample space MLT",
    "Accepted small steps", EPercentage);
StatsCounter rejectionRate("Primary sample space MLT",
    "Rejected steps", EPercentage);
StatsCounter domainRatio("Primary sample space MLT",
    "Rejected due to domain", EPercentage);
StatsCounter forcedAcceptance("Primary sample space MLT",
    "Number of forced acceptances");

class PSSMLTRenderer : public WorkProcessor {
public:
    PSSMLTRenderer(const MYPSSMLTConfiguration &conf, OutlierDetector const* outlierDetector)
        : m_config(conf), m_outlierDetector(outlierDetector) {
    }

    PSSMLTRenderer(Stream *stream, InstanceManager *manager)
        : WorkProcessor(stream, manager) {
        m_config = MYPSSMLTConfiguration(stream);
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        m_config.serialize(stream);
    }

    ref<WorkUnit> createWorkUnit() const {
        return new SeedWorkUnit();
    }

    ref<WorkResult> createWorkResult() const {
        return new ImageBlock(Bitmap::ESpectrum,
            m_film->getCropSize(), m_film->getReconstructionFilter());
    }

    void prepare() {

        // Log(EInfo, "Preparing");

        Scene *scene = static_cast<Scene*>(getResource("scene"));
        m_origSampler = static_cast<MyPSSMLTSampler*>(getResource("sampler"));
        m_sensor = static_cast<Sensor *>(getResource("sensor"));
        m_scene = new Scene(scene);
        m_film = m_sensor->getFilm();
        m_scene->setSensor(m_sensor);
        m_scene->setSampler(m_origSampler);
        m_scene->removeSensor(scene->getSensor());
        m_scene->addSensor(m_sensor);
        m_scene->setSensor(m_sensor);
        m_scene->wakeup(NULL, m_resources);
        m_scene->initializeBidirectional();

        m_rplSampler = static_cast<MyRplSampler*>(
            static_cast<Sampler *>(getResource("rplSampler"))->clone().get());
        m_sensorSampler = new MyPSSMLTSampler(m_origSampler);
        m_emitterSampler = new MyPSSMLTSampler(m_origSampler);
        m_directSampler = new MyPSSMLTSampler(m_origSampler);

        m_pathSampler = new PathSampler(m_config.technique, m_scene,
            m_emitterSampler, m_sensorSampler, m_directSampler, m_config.maxDepth,
            m_config.rrDepth, false, false);

        // Log(EInfo, "Prepartion Done");
        
    }

    void process(const WorkUnit *workUnit, WorkResult *workResult, const bool &stop) {

        // Log(EInfo, "Processing");

        ImageBlock *result = static_cast<ImageBlock *>(workResult);
        const PositionedSeedWorkUnit *wu = static_cast<const PositionedSeedWorkUnit*>(workUnit);
        const std::vector<PositionedPathSeed> &seeds = wu->getSeeds();
        SplatList *current = new SplatList(), *proposed = new SplatList();

        ref<Timer> timer = new Timer();

        result->clear();

        // Log(EInfo, "rplSamplerIndex: %i", m_rplSampler->getSampleIndex());

        for (auto const& seed : seeds) {

            /* Generate the initial sample by replaying the seeding random
            number stream at the appropriate position. Afterwards, revert
            back to this worker's own source of random numbers */

            m_rplSampler->reSeed(seed.seed);

            m_emitterSampler->reset();
            m_sensorSampler->reset();
            m_directSampler->reset();
            m_sensorSampler->setRandom(m_rplSampler->getRandom());
            m_emitterSampler->setRandom(m_rplSampler->getRandom());
            m_directSampler->setRandom(m_rplSampler->getRandom());

            m_rplSampler->setSampleIndex(seed.index +2); // +2 Because of setting the sensor samples manual.

            // Setting the sensor samples manual
            // This is needed because the MC samples where used within a pixel, here they are used over the whole image.
            ((MyPSSMLTSampler*) (m_pathSampler->getSensorSampler()))->setPrimarySample(0, seed.position.x);
            ((MyPSSMLTSampler*) (m_pathSampler->getSensorSampler()))->setPrimarySample(1, seed.position.y);

            // Log(EInfo, "position=[%f, %f]  index=%i", seed.position.x *200, seed.position.y *200, seed.sampleIndex);

            m_pathSampler->sampleSplats(Point2i(-1), *current);

            ref<Random> random = m_origSampler->getRandom();
            m_sensorSampler->setRandom(random);
            m_emitterSampler->setRandom(random);
            m_directSampler->setRandom(random);
            m_rplSampler->updateSampleIndex(m_rplSampler->getSampleIndex()
                + m_sensorSampler->getSampleIndex()
                + m_emitterSampler->getSampleIndex()
                + m_directSampler->getSampleIndex() -2); // -2 because of setting the first two manualy there have been used less from rplSampler

            m_sensorSampler->accept();
            m_emitterSampler->accept();
            m_directSampler->accept();

            /* Sanity check -- the luminance should match the one from
            the warmup phase - an error here would indicate inconsistencies
            regarding the use of random numbers during sample generation */
            if (std::abs((current->luminance - seed.luminance) / seed.luminance) > 0.1) {
                Log(EWarn, "Error when reconstructing a seed path (%i): luminance "
                    "= %f, but expected luminance = %f", seed.seed, current->luminance, seed.luminance);
                return;
            }

            // Log(EInfo, "seed luminance: %f, seed pdf: %f", seed.luminance, seed.pdf);

            m_sensorSampler->setLargeStep(false);
            m_emitterSampler->setLargeStep(false);
            m_directSampler->setLargeStep(false);

            // Log(EInfo, "Setup of mlt done");

            // Luminance correction f(u(0))/pmf(u(0)), see formula 9 in selective mlt
            auto correction = seed.luminance / seed.pdf;

            /* MLT main loop */
            Float cumulativeWeight = 0;
            ref<Bitmap> dummy;
            current->normalize(nullptr);
            for (uint64_t mutationCtr=0; mutationCtr<m_config.nMutations && !stop; ++mutationCtr) {
                if (wu->getTimeout() > 0 && (mutationCtr % 8192) == 0
                        && (int) timer->getMilliseconds() > wu->getTimeout())
                    break;

                m_pathSampler->sampleSplats(Point2i(-1), *proposed);
                // This does the normalisation of dividing by the luminance
                proposed->normalize(nullptr);

                // TODO: works only for unidirectional
                // Mlt integrates f(u) * w with w == 0 || w == 1
                Float w = m_outlierDetector->calculateWeight(proposed->getPosition(0), proposed->luminance);

                if (w == 0) {
                    ++domainRatio;
                }

                // multiply f(u) with w
                for (auto& splat : proposed->splats) {
                    splat.second *= w;
                }
                proposed->luminance *= w;

                Float a = std::min((Float) 1.0f, proposed->luminance / current->luminance);

                if (std::isnan(proposed->luminance) || proposed->luminance < 0) {
                    Log(EWarn, "Encountered a sample with luminance = %f, ignoring!",
                            proposed->luminance);
                    a = 0;
                }

                bool accept;
                Float currentWeight, proposedWeight;


                if (a > 0) {  
                    //TODO: look into kelemen style weights

                    currentWeight = 1-a;
                    proposedWeight = a;
                    
                    accept = (a == 1) || (random->nextFloat() < a);
                    
                } else {
                    currentWeight = 1;
                    proposedWeight = 0;
                    accept = false;
                    ++rejectionRate;
                    domainRatio.incrementBase(1);
                }
                rejectionRate.incrementBase(1);

                cumulativeWeight += currentWeight;

                if (accept) {
                    for (size_t k=0; k<current->size(); ++k) {
                        Spectrum value = current->getValue(k);
                        value *= correction * cumulativeWeight;
                        if (!value.isZero()) {
                            result->put(current->getPosition(k), &value[0]);
                        }
                    }

                    cumulativeWeight = proposedWeight;
                    std::swap(proposed, current);

                    m_sensorSampler->accept();
                    m_emitterSampler->accept();
                    m_directSampler->accept();
                
                    smallStepRatio.incrementBase(1);
                    ++smallStepRatio;

                } else {
                    for (size_t k=0; k<proposed->size(); ++k) {
                        Spectrum value = proposed->getValue(k);
                        value *= proposedWeight * correction;
                        if (!value.isZero()) {
                            result->put(proposed->getPosition(k), &value[0]);
                        }
                    }

                    m_sensorSampler->reject();
                    m_emitterSampler->reject();
                    m_directSampler->reject();
                    smallStepRatio.incrementBase(1);
                }
            }

            /* Perform the last splat */
            for (size_t k=0; k<current->size(); ++k) {
                Spectrum value = current->getValue(k);
                value *= correction * cumulativeWeight;
                if (!value.isZero()) {
                    result->put(current->getPosition(k), &value[0]);
                }
            }
            current->clear();
            proposed->clear();
        }

        delete current;
        delete proposed;
    }

    ref<WorkProcessor> clone() const {
        return new PSSMLTRenderer(m_config, m_outlierDetector);
    }

    MTS_DECLARE_CLASS()
private:
    MYPSSMLTConfiguration m_config;
    ref<Scene> m_scene;
    ref<Sensor> m_sensor;
    ref<Film> m_film;
    ref<PathSampler> m_pathSampler;
    ref<MyPSSMLTSampler> m_origSampler;
    ref<MyPSSMLTSampler> m_sensorSampler;
    ref<MyPSSMLTSampler> m_emitterSampler;
    ref<MyPSSMLTSampler> m_directSampler;
    ref<MyRplSampler> m_rplSampler;
    const OutlierDetector* m_outlierDetector;
};

/* ==================================================================== */
/*                           Parallel process                           */
/* ==================================================================== */

PSSMLTProcess::PSSMLTProcess(const RenderJob *parent, RenderQueue *queue,
    const MYPSSMLTConfiguration &conf,
    const std::vector<PositionedPathSeed> &seeds, Bitmap* mltResult, OutlierDetector const* outlierDetector) : m_job(parent), m_queue(queue),
        m_config(conf), m_progress(NULL), m_seeds(seeds), mlt_result(mltResult), m_outlierDetector(outlierDetector) {
    m_timeoutTimer = new Timer();
    m_refreshTimer = new Timer();
    m_resultMutex = new Mutex();
    m_resultCounter = 0;
    m_workCounter = 0;
    m_refreshTimeout = 1;
}

ref<WorkProcessor> PSSMLTProcess::createWorkProcessor() const {
    return new PSSMLTRenderer(m_config, m_outlierDetector);
}

void PSSMLTProcess::develop() {
    LockGuard lock(m_resultMutex);
    size_t pixelCount = m_accum->getBitmap()->getPixelCount();
    const Spectrum *accum = (Spectrum *) m_accum->getBitmap()->getData();
    Spectrum *target = (Spectrum *) mlt_result->getData();

    auto invBudget = 1.f/(m_seeds.size()*m_config.nMutations);

    for (size_t i=0; i<pixelCount; ++i) {
        // Normalise for the mlt budget (nb of mlt samples), see formula 9 selective mlt
        target[i] += accum[i] * invBudget;
    }

    m_refreshTimer->reset();
    m_queue->signalRefresh(m_job);
}

void PSSMLTProcess::processResult(const WorkResult *wr, bool cancelled) {
    LockGuard lock(m_resultMutex);
    const ImageBlock *result = static_cast<const ImageBlock *>(wr);
    m_accum->put(result);
    m_progress->update(++m_resultCounter);
    m_refreshTimeout = std::min(2000U, m_refreshTimeout * 2);

    /* Re-develop the entire image every two seconds if partial results are
       visible (e.g. in a graphical user interface). */
    if (m_job->isInteractive() && m_refreshTimer->getMilliseconds() > m_refreshTimeout)
        develop();
}

ParallelProcess::EStatus PSSMLTProcess::generateWork(WorkUnit *unit, int worker) {
    int timeout = 0;
    if (m_config.timeout > 0) {
        timeout = static_cast<int>(static_cast<int64_t>(m_config.timeout*1000) -
                  static_cast<int64_t>(m_timeoutTimer->getMilliseconds()));
    }

    if (m_workCounter >= m_config.workUnits || timeout < 0)
        return EFailure;

    PositionedSeedWorkUnit *workUnit = static_cast<PositionedSeedWorkUnit *>(unit);

    // compute the number of seeds the new workunit will work on
    int nSeeds = m_seeds.size() / m_config.workUnits;
    int rest = m_seeds.size() % m_config.workUnits;
    auto startSeed = nSeeds * m_workCounter;
    auto endSeed = startSeed + nSeeds;

    // We add one seed to the first rest workunits to have the best possible spread of work
    // This means that all subcequent places need to be shifted
    if (m_workCounter < rest) {
        endSeed += m_workCounter +1;
        startSeed += m_workCounter;
    } else {
        endSeed += rest;
        startSeed += rest;
    }

    assert(endSeed <= m_seeds.size());
    // Assign the seeds 
    auto seeds = std::vector<PositionedPathSeed>(m_seeds.begin() + startSeed, m_seeds.begin() + endSeed);

    ++m_workCounter;

    workUnit->setSeeds(seeds);
    workUnit->setTimeout(timeout);
    return ESuccess;
}

void PSSMLTProcess::bindResource(const std::string &name, int id) {
    ParallelProcess::bindResource(name, id);
    if (name == "sensor") {
        m_film = static_cast<Sensor *>(Scheduler::getInstance()->getResource(id))->getFilm();
        if (m_progress)
            delete m_progress;
        m_progress = new ProgressReporter("Rendering", m_config.workUnits, m_job);
        m_accum = new ImageBlock(Bitmap::ESpectrum, m_film->getCropSize());
        m_accum->clear();
    }
}

MTS_IMPLEMENT_CLASS_S(PSSMLTRenderer, false, WorkProcessor)
MTS_IMPLEMENT_CLASS(PSSMLTProcess, false, ParallelProcess)
MTS_IMPLEMENT_CLASS(SeedWorkUnit, false, WorkUnit)

MTS_NAMESPACE_END
