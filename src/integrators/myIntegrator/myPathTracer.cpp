

#include <string>
#include <typeinfo>

#include "myPathTracer.h"
#include <mitsuba/core/plugin.h>
#include "myrenderproc.h"
#include "utils/writeBitmap.h"

#include "MutatablePssmltSampler.h"
#include "my_pssmlt_proc.h"

#include <mitsuba/bidir/pathsampler.h>
#include <mitsuba/bidir/rsampler.h>


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
        m_config.directSampling = props.getBoolean("directSampling", false);
        m_config.mutationSizeLow  = props.getFloat("mutationSizeLow",  1.0f/1024.0f);
        m_config.mutationSizeHigh = props.getFloat("mutationSizeHigh", 1.0f/64.0f);
        Assert(m_config.mutationSizeLow > 0 && m_config.mutationSizeHigh > 0 &&
               m_config.mutationSizeLow < 1 && m_config.mutationSizeHigh < 1 &&
               m_config.mutationSizeLow < m_config.mutationSizeHigh);
        m_config.workUnits = props.getInteger("workUnits", -1);
        /* Stop MLT after X seconds -- useful for equal-time comparisons */
        m_config.timeout = props.getInteger("timeout", 0);

        iterations = props.getInteger("iterations", 10);

        seedMutex = new Mutex();
  }

MyPathTracer::MyPathTracer(Stream *stream, InstanceManager *manager)
 : Integrator(stream, manager) { 
        seedMutex = new Mutex();

  }

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
    Sampler *sampler = static_cast<Sampler *>(sched->getResource(samplerResID, 0));

    auto cropSize = film->getCropSize();
    invSize = Vector2(1.f/cropSize.x, 1.f/cropSize.y);

    samplesPerPixel = sampler->getSampleCount();
    samplesTotal = samplesPerPixel * cropSize.x * cropSize.y;

    ref<Bitmap> mltResult = new Bitmap(Bitmap::ESpectrum, Bitmap::EFloat, cropSize);

    if (sensor->needsApertureSample())
        Log(EError, "No support for aperture samples at this time!");
    if (sensor->needsTimeSample())
        Log(EError, "No support for time samples at this time!");

    detector = new OutlierDetectorBitterly(cropSize.x, cropSize.y, 8, 0.5, 2, 1000);


    Log(EInfo, "Starting render job (%ix%i, " SIZE_T_FMT " %s, " SIZE_T_FMT
        " %s, " SSE_STR ") ..", cropSize.x, cropSize.y,
        samplesPerPixel, samplesPerPixel == 1 ? "sample" : "samples", nCores,
        nCores == 1 ? "core" : "cores");

    ref<Bitmap> directImage;

    // mlt samplers
    ref<MutatablePSSMLTSampler> mltSampler = new MutatablePSSMLTSampler(m_config);
    std::vector<SerializableObject *> mltSamplers(sched->getCoreCount());
    for (size_t i=0; i<mltSamplers.size(); ++i) {
        ref<Sampler> clonedSampler = mltSampler->clone();
        clonedSampler->incRef();
        mltSamplers[i] = clonedSampler.get();
    }
    int mltSamplerResID = sched->registerMultiResource(mltSamplers);
    for (size_t i=0; i<sched->getCoreCount(); ++i)
        mltSamplers[i]->decRef();

    // path tracer samplers
    ref<ReplayableSampler> rplSampler = new ReplayableSampler();
    std::vector<SerializableObject*> rplSamplers(sched->getCoreCount());
    for (size_t i=0; i<rplSamplers.size(); ++i) {
        ref<Sampler> clonedSampler = rplSampler->clone();
        clonedSampler->incRef();
        rplSamplers[i] = clonedSampler.get();
    }
    int rplSamplerResID = sched->registerMultiResource(rplSamplers);
    for (size_t i=0; i<sched->getCoreCount(); ++i)
        rplSamplers[i]->decRef();


    // Set chain lenght
    m_config.nMutations = 1000;

    int integratorResID = sched->registerResource(this);

    for (iteration=0; iteration<iterations; ++iteration) {

        Log(EInfo, "Starting on path tracing in iteration %i", iteration);

        ref<ParallelProcess> proc = new BlockedRenderProcess(job, queue, scene->getBlockSize());
        
        proc->bindResource("integrator", integratorResID);
        proc->bindResource("scene", sceneResID);
        proc->bindResource("sensor", sensorResID);
        proc->bindResource("sampler", rplSamplerResID);
        scene->bindUsedResources(proc);
        bindUsedResources(proc);
        sched->schedule(proc);

        m_process = proc;
        sched->wait(proc);
        m_process = NULL;
        

        if (proc->getReturnStatus() != ParallelProcess::ESuccess) {
            Log(EError, "Error while path tracing.");
        }

        if (iteration != 0) {
            Float avgLuminance = 0;
            for (auto& seed : pathSeeds) {
                avgLuminance += seed.luminance;
            }
            size_t nb_seeds = pathSeeds.size();
            avgLuminance /= nb_seeds;
            m_config.luminance = avgLuminance;



            if (nb_seeds > 0) {
                // mlt budget is nb chains * nb mutations
                auto mltBudget = computeMltBudget();
                auto nbOfChains = mltBudget / m_config.nMutations;
                m_config.workUnits = nbOfChains;

                // Pick seeds proportional to their luminance
                auto seeds = drawSeeds(pathSeeds, nbOfChains, sampler);
                // update the detector for the next iteration.
                detector->update(pathSeeds, nbOfChains);

                // Every seed is used by one work unit.
                assert(seeds.size() == (size_t) m_config.workUnits);

                // We dont need the original list any more. The seeds that will be used are copied to seeds.
                pathSeeds.clear();
            
                Log(EInfo, "Starting on mlt in iteration %i with %i seeds. Avg luminance is %f.", iteration, m_config.workUnits, avgLuminance);

                ref<PSSMLTProcess> process = new PSSMLTProcess(job, queue, m_config, directImage, seeds, mltBudget, mltResult);
                process->bindResource("scene", sceneResID);
                process->bindResource("sensor", sensorResID);
                process->bindResource("sampler", mltSamplerResID);
                process->bindResource("rplSampler", rplSamplerResID);

                // Log(EInfo, "Binded resources");

                m_process = process;
                sched->schedule(process);
                sched->wait(process);
                m_process = NULL;
                
                process->develop();


                if (proc->getReturnStatus() != ParallelProcess::ESuccess) {
                    Log(EError, "Error while mlting.");
                }    
            }
        }
    }
    sched->unregisterResource(rplSamplerResID);
    sched->unregisterResource(integratorResID);

    mltResult->scale(1.f/iterations);
    film->addBitmap(mltResult);

    BitmapWriter::writeBitmap(mltResult, BitmapWriter::EHDR, "/mnt/c/Users/beast/Documents/00-School/master/thesis/prentjes/first-tests/mlt.png");
    
    return true;
}

void MyPathTracer::bindUsedResources(ParallelProcess *) const {
    /* Do nothing by default */
}

void MyPathTracer::wakeup(ConfigurableObject *parent,
    std::map<std::string, SerializableObject *> &) {
    /* Do nothing by default */
}

size_t MyPathTracer::computeMltBudget() const {
    Float r = weightedAvg.get() / unweightedAvg.get();
    return samplesTotal * r / (1-r);
}

void MyPathTracer::renderBlock(const Scene *scene,
        const Sensor *sensor, Sampler *sampler, ImageBlock *block,
        const bool &stop, const std::vector<TPoint2<uint8_t>> &points) {

    SplatList splatList;
    ref<PathSampler> pathSampler = new PathSampler(m_config.technique, scene,
            sampler, sampler, sampler, m_config.maxDepth, m_config.rrDepth,
            false, false, false);

    std::vector<PositionedPathSeed> localPathSeeds;
    size_t nb_seeds = 0;

    block->clear();

    // for (size_t i = 0; i<1; ++i) {
    for (size_t i = 0; i<points.size(); ++i) {
        Point2i offset = Point2i(points[i]) + Vector2i(block->getOffset());
        if (stop)
            break;

        sampler->generate(offset);

        for (size_t j = 0; j<samplesPerPixel; j++) {

            splatList.clear();
            auto index = sampler->getSampleIndex();
            sampler->setSampleIndex(index);

            pathSampler->sampleSplats(offset, splatList);

            auto spec = splatList.splats[0].second;
            auto position = splatList.splats[0].first;
            auto luminance = splatList.luminance;

            // // Log(EInfo, "Index: %i    Luminance: %f", index, splatList.splats[0].second.getLuminance());

            detector->contribute(offset, luminance);
            auto weight = detector->calculateWeight(offset, luminance, iteration*samplesPerPixel);

            weightedAvg.put(weight*luminance);
            unweightedAvg.put(luminance);


            if (iteration != 0) {
                block->put(position, spec * (1-weight), 1); //TODO: scaling of samples

                if (weight > 0) {
                    // localPathSeeds.emplace_back(Point2(position.x * invSize.x, position.y * invSize.y), index, luminance);
                    localPathSeeds.push_back(PositionedPathSeed(Point2(position.x * invSize.x, position.y * invSize.y), index, luminance));
                    // Log(EInfo, "Position=[%f,%f]  index=%i", position.x, position.y, index);
                    nb_seeds++;
                } 
            }

            sampler->advance();
        }
    }
    if (!localPathSeeds.empty()) {
        LockGuard lock(seedMutex); // pathSeeds is global
        pathSeeds.insert( pathSeeds.end(), localPathSeeds.begin(), localPathSeeds.end() );
    }
}

MTS_IMPLEMENT_CLASS(MyPathTracer, false, Integrator);
MTS_EXPORT_PLUGIN(MyPathTracer, "myPathTracer");

MTS_NAMESPACE_END
