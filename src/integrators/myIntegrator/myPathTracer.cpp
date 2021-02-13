

#include "myPathTracer.h"
#include <mitsuba/core/plugin.h>
#include "myrenderproc.h"

#include <mitsuba/bidir/pathsampler.h>

MTS_NAMESPACE_BEGIN

MyPathTracer::MyPathTracer(const Properties &props)
 : Integrator(props) { }

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

    /* This is a sampling-based integrator - parallelize */
    ref<ParallelProcess> proc = new BlockedRenderProcess(job,
        queue, scene->getBlockSize());
    int integratorResID = sched->registerResource(this);
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
    sched->unregisterResource(integratorResID);

    return proc->getReturnStatus() == ParallelProcess::ESuccess;

    // Here comes the MLT stuff
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
