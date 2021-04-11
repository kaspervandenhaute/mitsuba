
#include "myPathTracer.h"

#include <string>
#include <typeinfo>
#include <stdio.h>
#include <numeric>

#include <mitsuba/core/plugin.h>
#include <mitsuba/bidir/pathsampler.h>

#include "myrenderproc.h"

#include "my_pssmlt_proc.h"


#include "outlierDetectors/bitterli.h"
#include "outlierDetectors/zirr1.h"
#include "outlierDetectors/testDetector.h"
#include "outlierDetectors/thresholdDetector.h"

#define THESISLOCATION std::string("/mnt/c/Users/beast/Documents/00-school/master/thesis/")
// #define THESISLOCATION std::string("/mnt/g/Documents/00-school/master/thesis/")


MTS_NAMESPACE_BEGIN

StatsCounter seedCounter("My integrator", "Number of seeds");
StatsCounter outlierCounter("My integrator", "Number of outliers");
StatsCounter inlierMinValue("My integrator", "Inliers with higher luminance than minValue", EPercentage);


MyPathTracer::MyPathTracer(const Properties &props)
 : Integrator(props), props(props) {

    // Test options
    nPoints = props.getInteger("points", 20);
    nSubPoints = props.getInteger("subpoints", 10);
    
    testProperty = props.getString("testProperty", "iterations");
    exponential = props.getBoolean("exponentialTest", true);
    minValue = props.getFloat("minValue", 0);
    maxValue = props.getFloat("maxValue", 1000);

    init();

  }


void MyPathTracer::renderSetup(Scene *scene,
        RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {
    sched = Scheduler::getInstance();
    sensor = static_cast<Sensor *>(sched->getResource(sensorResID));
    film = sensor->getFilm();

    nCores = sched->getCoreCount();
    sampler = static_cast<Sampler *>(sched->getResource(samplerResID, 0));

    cropSize = film->getCropSize();
    invSize = Vector2(1.f/cropSize.x, 1.f/cropSize.y);

    samplesTotal = samplesPerPixel * cropSize.x * cropSize.y;

    mltResult = new Bitmap(Bitmap::ESpectrum, Bitmap::EFloat, cropSize);
    pathResult = new ImageBlock(Bitmap::ESpectrum, cropSize, film->getReconstructionFilter());
    seedsResult = new ImageBlock(Bitmap::ESpectrum, cropSize, film->getReconstructionFilter());
    outliersResult = new ImageBlock(Bitmap::ESpectrum, cropSize, film->getReconstructionFilter());
    outlierDomain = new ImageBlock(Bitmap::ESpectrum, cropSize, film->getReconstructionFilter());

    if (sensor->needsApertureSample())
        Log(EError, "No support for aperture samples at this time!");
    if (sensor->needsTimeSample())
        Log(EError, "No support for time samples at this time!");

    Log(EInfo, "Starting render job (%ix%i, " SIZE_T_FMT " %s, " SIZE_T_FMT
        " %s, " SSE_STR ") ..", cropSize.x, cropSize.y,
        samplesPerPixel, samplesPerPixel == 1 ? "sample" : "samples", nCores,
        nCores == 1 ? "core" : "cores");

}

void MyPathTracer::pathTracing(Scene *scene, RenderQueue *queue, const RenderJob *job, 
    int sceneResID, int sensorResID, int samplerResID, int rplSamplerResID, int integratorResID) {

    // Log(EInfo, "Starting on path tracing in iteration %i", iteration);

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

    samplesTotal = samplesPerPixel * cropSize.x * cropSize.y;
    cost += samplesTotal;
    

    if (proc->getReturnStatus() != ParallelProcess::ESuccess) {
        Log(EError, "Error while path tracing.");
    }
}


void MyPathTracer::initDetector(Scene *scene, RenderQueue *queue, const RenderJob *job, 
    int sceneResID, int sensorResID, int samplerResID, int rplSamplerResID, int integratorResID) {

    iteration = 0;
    
    // int spp = samplesPerPixel;
    // samplesPerPixel = 1;

    // for (int i=0; i<spp; ++i) {
    //     pathTracing(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID); 
    //     detector->update(pathSeeds, computeMltBudget()/m_config.nMutations, i);
    //     pathSeeds.clear();
    // }
    // weightedAvg.reset();
    // unweightedAvg.reset();

    pathTracing(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID);
    detector->update(samplesPerPixel);
    pathSeeds.clear();

    cost += samplesTotal;

    // Set the spp back to the requested value after initialisation
    samplesPerPixel = sampler->getSampleCount();
}


bool MyPathTracer::myRender(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {

    int mltSamplerResID, rplSamplerResID;
    initialiseSamplers(mltSamplerResID, rplSamplerResID);

    // detector = new OutlierDetectorBitterly(cropSize.x, cropSize.y, 16, 0.5, 2, 300);
    detector = new OutlierDetectorZirr1(cropSize.x, cropSize.y, 2, 300, kappa, outlierDetectorThreshold);
    // detector = new ThresholdDetector();
    // detector = new TestOutlierDetector();

    int integratorResID = sched->registerResource(this);

    // Initialise the outlier detector
    initDetector(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID);

    for (iteration=1; iteration<iterations; ++iteration) {

        pathTracing(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID);

        size_t nbOfChains = 0;

        MltStats mltStats;

        // mlt budget is nb chains * nb mutations
        auto mltBudget = computeMltBudget();
        // if there are seeds, samples have been discarded. They need to be put back
        nbOfChains = pathSeeds.size() > 0 ? std::max(mltBudget / m_config.nMutations, (size_t) 1) : 0;

        // actual mlt budget
        mltBudget = nbOfChains * m_config.nMutations;

        cost += mltBudget;

        if (nbOfChains > 0) {
            m_config.workUnits = std::min(nCores == 1 ? 1 : nCores*4, nbOfChains);

            // Pick seeds proportional to their luminance
            auto seeds = drawSeeds(pathSeeds, nbOfChains, sampler);

            updateOutlierSeedsStats(pathSeeds, seeds, outlierCounter, seedCounter);

            // Multiple seeds per work unit
            assert(seeds.size() >= (size_t) m_config.workUnits);

            Float avgLuminance = std::accumulate(seeds.begin(), seeds.end(), 0, [] (Float accum, PositionedPathSeed const& seed1) 
                                                            {return accum + seed1.luminance;} ) / seeds.size();
        
            Log(EInfo, "Starting on mlt in iteration %i with %i seeds out of %i candidates. Avg luminance is %f.", iteration, nbOfChains, pathSeeds.size(), avgLuminance);


            ref<PSSMLTProcess> process = new PSSMLTProcess(job, queue, m_config, seeds, mltResult, detector);
            process->bindResource("scene", sceneResID);
            process->bindResource("sensor", sensorResID);
            process->bindResource("sampler", mltSamplerResID);
            process->bindResource("rplSampler", rplSamplerResID);

            // Log(EInfo, "Binded resources");
            mltStats = process->getMltStats();

            m_process = process;
            sched->schedule(process);
            sched->wait(process);
            m_process = NULL;
            
            process->develop();


            if (process->getReturnStatus() != ParallelProcess::ESuccess) {
                Log(EError, "Error while mlting.");
            }    
        }     

        writeStatisticsToFile(pathSeeds.size(), nbOfChains, mltStats);

        // reset r
        weightedAvg.reset();        
        unweightedAvg.reset();        

        // update the detector for the next iteration.
        detector->update(pathSeeds, nbOfChains, samplesPerPixel);
        pathSeeds.clear();

        if (intermediatePeriod > 0 && iteration != 0 && iteration % intermediatePeriod == 0) {
            writeAvos(THESISLOCATION + "prentjes/test/avos/", std::to_string(iteration));
        }
    }

    if (intermediatePeriod != 0) {
        writeTotal( intermediatePath + std::to_string(cost/1000) + ".exr");
    } else {
        auto result = createResult();
        film->setBitmap(result);
    }

    if (intermediatePeriod != -1) {
        mltResult->scale(1.f/samplesPerPixel); //TODO: Why?
        writeAvos(THESISLOCATION + "prentjes/test/");
    }

    clearResults();

    cost = 0;

    sched->unregisterResource(rplSamplerResID);
    sched->unregisterResource(integratorResID);
    
    return true;
}


void MyPathTracer::renderBlock(const Scene *scene,
        const Sensor *sensor, Sampler *_sampler, ImageBlock *block,
        const bool &stop, const std::vector<TPoint2<uint8_t>> &points) {

    auto* sampler = (MyRplSampler*) _sampler;
    
    ref<PathSampler> pathSampler = new PathSampler(m_config.technique, scene,
            sampler, sampler, sampler, m_config.maxDepth, m_config.rrDepth,
            false, false, false);

    std::vector<PositionedPathSeed> localPathSeeds;

    block->clear();

    SplatList splatList;

    float weighted = 0;
    float unweighted = 0;

    auto invSpp = 1.f/samplesPerPixel;

    bool extra = false;


    for (size_t i = 0; i<points.size(); ++i) {
        Point2i offset = Point2i(points[i]) + Vector2i(block->getOffset());
        if (stop)
            break;

        // hash function is only needed for repeatability
        // auto seed = createSeed(offset);
        auto seed = random->nextULong();
        sampler->reSeed(seed);

        sampler->generate(offset);

        for (size_t j = 0; j<samplesPerPixel; j++) {

            if (j >= samplesPerPixel) {
                extra = true;
            }

            size_t index = sampler->getSampleIndex();
            
            splatList.clear();
            pathSampler->sampleSplats(offset, splatList);

            auto spec = splatList.splats[0].second;
            auto position = splatList.splats[0].first;
            auto luminance = splatList.luminance;

            // Log(EInfo, "Index: %i    Luminance: %f", index, splatList.splats[0].second.getLuminance());
            
            detector->contribute(position, luminance);

            if (iteration != 0 && !extra) {
            
                auto weight = detector->calculateWeight(position, luminance); // TODO not thread save

                weighted += weight*luminance;
                unweighted += luminance;

                if (weight > 0) {
                    localPathSeeds.emplace_back(Point2((double) position.x / cropSize.x, (double) position.y / cropSize.y), seed, index, luminance, spec);
                } else {
                    inlierMinValue.incrementBase();
                    if (detector->minValue < luminance) {
                        ++inlierMinValue;
                    }
                }
                block->put(position, spec * (1-weight) * invSpp, 1);              
            }

            sampler->advance();
        }
    }

    LockGuard lock(seedMutex); // pathSeeds and pathResult are shared by all threads
    pathResult->put(block);

    auto count = points.size() * samplesPerPixel;

    weightedAvg.put(weighted, count);
    unweightedAvg.put(unweighted, count);

    if (!localPathSeeds.empty()) {
        pathSeeds.insert( pathSeeds.end(), localPathSeeds.begin(), localPathSeeds.end() );
    }
}

void MyPathTracer::init() {
        m_config.maxDepth = props.getInteger("maxDepth", -1);
        m_config.rrDepth = props.getInteger("rrDepth", 5);
        m_config.technique = props.getBoolean("bidirectional", false) ?
            PathSampler::EBidirectional : PathSampler::EUnidirectional;

        m_config.mutationSizeLow  = props.getFloat("mutationSizeLow",  1.0f/1024.0f);
        m_config.mutationSizeHigh = props.getFloat("mutationSizeHigh", 1.0f/64.0f);
        Assert(m_config.mutationSizeLow > 0 && m_config.mutationSizeHigh > 0 &&
               m_config.mutationSizeLow < 1 && m_config.mutationSizeHigh < 1 &&
               m_config.mutationSizeLow < m_config.mutationSizeHigh);
        m_config.workUnits = props.getInteger("workUnits", -1);
        /* Stop MLT after X seconds -- useful for equal-time comparisons */
        m_config.timeout = props.getInteger("timeout", 0);
        kappa = props.getInteger("kappa", 1);

        iterations = props.getInteger("iterations", 10);
        outlierDetectorThreshold = props.getFloat("outlierThreshold", 1);
        intermediatePeriod = props.getInteger("intermediatePeriod", 0);
        // -1 means testing for graphs, zero means standard operation
        if (intermediatePeriod > 0 || intermediatePeriod == -1) {
            intermediatePath = props.getString("intermediatePath");
        }

        samplesPerPixel = props.getInteger("samplesFirst", 1);

        // Set chain lenght
        m_config.nMutations = 1000;

        random = new Random();
        seedMutex = new Mutex();
}

bool MyPathTracer::render(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {

    renderSetup(scene, queue, job, sceneResID, sensorResID, samplerResID);


    if (props.getInteger("intermediatePeriod", 0) == -1) {

        for (int loop=0; loop<nPoints; ++loop) {
            float testValue;
            if (exponential) {
                testValue = minValue + std::pow(10, loop * std::log(maxValue-minValue)/std::log(10) /nPoints);
            } else {
                testValue = minValue + loop * (maxValue - minValue)/nPoints;
            }
            
            // Override the property we want to test
            props.removeProperty(testProperty);
            props.setInteger(testProperty, testValue);

            init();

            for (int i=0; i<nSubPoints; ++i) {               

                intermediatePath = props.getString("intermediatePath") + std::to_string(loop) + "-" + std::to_string(i) + "-";

                myRender(scene, queue, job, sceneResID, sensorResID, samplerResID);
            }
        }
    } else {
        init();
        myRender(scene, queue, job, sceneResID, sensorResID, samplerResID);
    }

    return true;
}

 void MyPathTracer::writeStatisticsToFile(int nOutliers, int nSeeds, MltStats mltStats) const {

    std::ofstream myfile (THESISLOCATION + "prentjes/test/stat_output.txt", std::ios::app);
    if (myfile.is_open())
    {
        if (iteration == 0) {
            myfile << "[\n";
        }

        myfile << "{";

        myfile << "\"iteration\": " << iteration << ", ";
        myfile << "\"nOutliers\": " << nOutliers << ", ";
        myfile << "\"nInliers\": " << samplesTotal - nOutliers << ", ";
        myfile << "\"nInliersMinValue\": " << inlierMinValue.getValue() << ", ";
        myfile << "\"nSeeds\": " << nSeeds << ", ";
        myfile << "\"cost\": " << cost << ", ";
        myfile << "\"r\": " << weightedAvg.get() / unweightedAvg.get() << ", ";
        myfile << "\"minValue\": " << detector->minValue << ", ";
        myfile << "\"nMutations\": " << mltStats.nMutations << ", ";
        myfile << "\"nRejections\": " << mltStats.nRejections << ", ";
        myfile << "\"nRejectionDomain\": " << mltStats.nRejectionDomain << ", ";
        myfile << "\"nRejectionsMinThresh\": " << mltStats.nRejectionDomainMinValue;
        
        myfile << "}";

        if (iteration != iterations-1) {
            myfile << ",\n";
        } else {
            myfile << "]";
        }

        myfile.close();
        
    } else {
        Log(EWarn, "Cant open statistics file. Not outputting statistics.");
    };
}

MTS_IMPLEMENT_CLASS(MyPathTracer, false, Integrator);
MTS_EXPORT_PLUGIN(MyPathTracer, "myPathTracer");

MTS_NAMESPACE_END
