

#include "myPathTracer.h"
#include <mitsuba/core/plugin.h>
#include "myrenderproc.h"

#include <mitsuba/bidir/pathsampler.h>

MTS_NAMESPACE_BEGIN

MyPathTracer::MyPathTracer(const Properties &props)
 : Integrator(props) {
        m_config.maxDepth = props.getInteger("maxDepth", -1);
        m_config.rrDepth = props.getInteger("rrDepth", 5);
        m_config.technique = props.getBoolean("bidirectional", false) ?
            PathSampler::EBidirectional : PathSampler::EUnidirectional;
        m_config.twoStage = props.getBoolean("twoStage", false);
        m_config.firstStageSizeReduction = props.getInteger(
            "firstStageSizeReduction", 16);
        m_config.firstStage= props.getBoolean("firstStage", false);
        m_config.luminanceSamples = props.getInteger("luminanceSamples", 0);
        m_config.pLarge = props.getFloat("pLarge", 0.f);
        m_config.directSamples = props.getInteger("directSamples", -1);
        m_config.separateDirect = m_config.directSamples >= 0;
        m_config.kelemenStyleWeights = props.getBoolean("kelemenStyleWeights", true);
        m_config.directSampling = props.getBoolean(
                "directSampling", false);
        m_config.mutationSizeLow  = props.getFloat("mutationSizeLow",  1.0f/1024.0f);
        m_config.mutationSizeHigh = props.getFloat("mutationSizeHigh", 1.0f/64.0f);
        Assert(m_config.mutationSizeLow > 0 && m_config.mutationSizeHigh > 0 &&
               m_config.mutationSizeLow < 1 && m_config.mutationSizeHigh < 1 &&
               m_config.mutationSizeLow < m_config.mutationSizeHigh);
        m_config.workUnits = props.getInteger("workUnits", -1);
        /* Stop MLT after X seconds -- useful for equal-time comparisons */
        m_config.timeout = props.getInteger("timeout", 0);
  }

MyPathTracer::MyPathTracer(Stream *stream, InstanceManager *manager)
 : Integrator(stream, manager) { }

void MyPathTracer::serialize(Stream *stream, InstanceManager *manager) const {
    Integrator::serialize(stream, manager);
}

void MyPathTracer::cancel() {
    if (m_process)
        Scheduler::getInstance()->cancel(m_process);
}

bool MyPathTracer::render(Scene *scene,
        RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {
    ref<Scheduler> sched = Scheduler::getInstance();
    ref<Sensor> sensor = static_cast<Sensor *>(sched->getResource(sensorResID));
    ref<Film> film = sensor->getFilm();

    size_t nCores = sched->getCoreCount();
    const Sampler *sampler = static_cast<const Sampler *>(sched->getResource(samplerResID, 0));
    size_t sampleCount = sampler->getSampleCount();

    Log(EInfo, "Starting render job (%ix%i, " SIZE_T_FMT " %s, " SIZE_T_FMT
        " %s, " SSE_STR ") ..", film->getCropSize().x, film->getCropSize().y,
        sampleCount, sampleCount == 1 ? "sample" : "samples", nCores,
        nCores == 1 ? "core" : "cores");

    // MLT setup with rplSampler and stuff

     if (m_config.workUnits <= 0) {
            const size_t cropArea  = (size_t) cropSize.x * cropSize.y;
            const size_t workUnits = ((desiredMutationsPerWorkUnit - 1) +
                (cropArea * sampleCount)) / desiredMutationsPerWorkUnit;
            Assert(workUnits <= (size_t) std::numeric_limits<int>::max());
            m_config.workUnits = (int) std::max(workUnits, (size_t) 1);
        }

    /* Create a sampler instance for each worker */
    ref<PSSMLTSampler> mltSampler = new PSSMLTSampler(m_config);
    std::vector<SerializableObject *> mltSamplers(scheduler->getCoreCount());
    for (size_t i=0; i<mltSamplers.size(); ++i) {
        ref<Sampler> clonedSampler = mltSampler->clone();
        clonedSampler->incRef();
        mltSamplers[i] = clonedSampler.get();
    }
    int mltSamplerResID = scheduler->registerMultiResource(mltSamplers);
    for (size_t i=0; i<scheduler->getCoreCount(); ++i)
        mltSamplers[i]->decRef();

    m_config.nMutations = (cropSize.x * cropSize.y *
    sampleCount) / m_config.workUnits;

    ref<ReplayableSampler> rplSampler = new ReplayableSampler();


    int integratorResID = sched->registerResource(this);

    for (int iter=0; iter<10; ++iter) {
        /* This is a sampling-based integrator - parallelize */
        ref<ParallelProcess> proc = new BlockedRenderProcess(job, queue, scene->getBlockSize());
        
        proc->bindResource("integrator", integratorResID);
        proc->bindResource("scene", sceneResID);
        proc->bindResource("sensor", sensorResID);
        proc->bindResource("sampler", samplerResID);
        scene->bindUsedResources(proc);
        bindUsedResources(proc);
        sched->schedule(proc);

        m_process = proc;
        sched->wait(proc);
        m_process = NULL;
        

        if (proc->getReturnStatus() != ParallelProcess::ESuccess) {
            Log(EError, "Error while path tracing.");
        }
     

        ref<PSSMLTProcess> process = new PSSMLTProcess(job, queue, m_config, directImage, pathSeeds);
        int rplSamplerResID = scheduler->registerResource(rplSampler);
        process->bindResource("scene", sceneResID);
        process->bindResource("sensor", sensorResID);
        process->bindResource("sampler", mltSamplerResID);
        process->bindResource("rplSampler", rplSamplerResID);

        m_process = process;
        scheduler->schedule(process);
        scheduler->wait(process);
        m_process = NULL;
        scheduler->unregisterResource(rplSamplerResID);
        process->develop();

        if (proc->getReturnStatus() != ParallelProcess::ESuccess) {
            Log(EError, "Error while mlting.");
        }    
    }
    sched->unregisterResource(integratorResID);

    return true;
}

void MyPathTracer::bindUsedResources(ParallelProcess *) const {
    /* Do nothing by default */
}

void MyPathTracer::wakeup(ConfigurableObject *parent,
    std::map<std::string, SerializableObject *> &) {
    /* Do nothing by default */
}

void MyPathTracer::renderBlock(const Scene *scene,
        const Sensor *sensor, Sampler *sampler, ImageBlock *block,
        const bool &stop, const std::vector< TPoint2<uint8_t> > &points) const {

    // Float diffScaleFactor = 1.0f /
    //     std::sqrt((Float) sampler->getSampleCount());

    // bool needsApertureSample = sensor->needsApertureSample();
    // bool needsTimeSample = sensor->needsTimeSample();

    // RadianceQueryRecord rRec(scene, sampler);
    // Point2 apertureSample(0.5f);
    // Float timeSample = 0.5f;
    // RayDifferential sensorRay;

// TODO: read parameters
    SplatList list;
    ref<PathSampler> pathSampler = new PathSampler(PathSampler::EUnidirectional, scene,
            sampler, sampler, sampler, 10, 5,
            false, true);

    block->clear();

    // uint32_t queryType = RadianceQueryRecord::ESensorRay;

    // if (!sensor->getFilm()->hasAlpha()) /* Don't compute an alpha channel if we don't have to */
    //     queryType &= ~RadianceQueryRecord::EOpacity;

    for (size_t i = 0; i<points.size(); ++i) {
        Point2i offset = Point2i(points[i]) + Vector2i(block->getOffset());
        if (stop)
            break;

        sampler->generate(offset);

        // TODO: get sample count (not from rplSampler)
        for (size_t j = 0; j<sampler->getSampleCount(); j++) {
            // rRec.newQuery(queryType, sensor->getMedium());
            // Point2 samplePos(Point2(offset) + Vector2(rRec.nextSample2D()));

            // if (needsApertureSample)
            //     apertureSample = rRec.nextSample2D();
            // if (needsTimeSample)
            //     timeSample = rRec.nextSample1D();

            // Spectrum spec = sensor->sampleRayDifferential(
            //     sensorRay, samplePos, apertureSample, timeSample);

            // sensorRay.scaleDifferential(diffScaleFactor);

            //TODO: PathSampler
            list.clear();
            pathSampler->sampleSplats(offset, list);
            // spec *= Spectrum(0.5);
            block->put(list.splats[0].first, list.splats[0].second, 1);
            sampler->advance();
        }
    }
}

MTS_IMPLEMENT_CLASS(MyPathTracer, false, Integrator);
MTS_EXPORT_PLUGIN(MyPathTracer, "myPathTracer");

MTS_NAMESPACE_END
