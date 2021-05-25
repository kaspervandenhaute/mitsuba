
#include "myPathTracer.h"

#include <string>
#include <typeinfo>
#include <stdio.h>


#include <mitsuba/core/plugin.h>
#include <mitsuba/bidir/pathsampler.h>

#include "myrenderproc.h"

#include "my_pssmlt_proc.h"

#include "utils/writeToBinary.h"


#include "outlierDetectors/bitterli.h"
#include "outlierDetectors/zirr1.h"
#include "outlierDetectors/testDetector.h"
#include "outlierDetectors/thresholdDetector.h"
#include "outlierDetectors/meanDetector.h"

#include "utils/thesisLocation.h"


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

    mltResult->clear();
    pathResult->clear();
    seedsResult->clear();
    outliersResult->clear();
    outlierDomain->clear();


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

    pathTracing(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID);
    detector->update(samplesPerPixel);
    pathSeeds.clear();

    // Set the spp back to the requested value after initialisation
    samplesPerPixel = sampler->getSampleCount();
}


bool MyPathTracer::myRender(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {

    int mltSamplerResID, rplSamplerResID;
    initialiseSamplers(mltSamplerResID, rplSamplerResID);

    int integratorResID = sched->registerResource(this);

    // size_t size = cropSize.x * cropSize.y * 8;
    // float* data = new float [size];
    // readBinaryFile(THESISLOCATION + "prentjes/test/bed_bufferBitterli100x100_10000spp.bin", data, size);

    // auto tempDetector = std::unique_ptr<OutlierDetectorBitterly>(new OutlierDetectorBitterly(cropSize.x, cropSize.y, 8, data, 10000, 0.5, 2, std::numeric_limits<float>::infinity(), outlierDetectorThreshold, additionalThresholding));
    
    // auto tempDetector = std::unique_ptr<OutlierDetectorBitterly>(new OutlierDetectorBitterly(cropSize.x, cropSize.y, 8, 0.5, 2, 1000, outlierDetectorThreshold, additionalThresholding));
    
    // auto tempDetector = std::unique_ptr<OutlierDetectorZirr1>(new OutlierDetectorZirr1(cropSize.x, cropSize.y, 2, 300, kappa, outlierDetectorThreshold));
    // auto tempDetector = std::unique_ptr<ThresholdDetector>(new ThresholdDetector());
    // auto tempDetector = std::unique_ptr<TestOutlierDetector>(new TestOutlierDetector());
    // auto tempDetector = std::unique_ptr<MeanOutlierDetector>(new MeanOutlierDetector(cropSize.x, cropSize.y, iterations * samplesPerPixel, outlierDetectorThreshold));
    
    assert(detectorSoftness == 0.f);
    if (detectorType == "bitterli") {
        detector = new SoftDetector(std::unique_ptr<OutlierDetectorBitterly>(new OutlierDetectorBitterly(cropSize.x, cropSize.y, 8, 0.5, 2, 1000, 1.f, additionalThresholding)), detectorSoftness);
        // assert(outlierDetectorThreshold == 1.f);
        initDetector(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID);
    }
    if (detectorType == "mean") {
        detector = new SoftDetector(std::unique_ptr<MeanOutlierDetector>(new MeanOutlierDetector(cropSize.x, cropSize.y, iterations * samplesPerPixel, 2.f)), detectorSoftness);
        // assert(outlierDetectorThreshold == 2.f);
        initDetectorMean(scene, sampler);
    }
    if (detectorType == "all-in") {
        detector = new SoftDetector(std::unique_ptr<TestOutlierDetector>(new TestOutlierDetector(0.f)), detectorSoftness);
    }
    if (detectorType == "all-out") {
        detector = new SoftDetector(std::unique_ptr<TestOutlierDetector>(new TestOutlierDetector(1.f)), detectorSoftness);
    }


    for (iteration=1; iteration<iterations; ++iteration) {

        pathTracing(scene, queue, job, sceneResID, sensorResID, samplerResID, rplSamplerResID, integratorResID);

        size_t nbOfChains = 0;

        MltStats mltStats;

        // mlt budget is nb chains * nb mutations
        auto mltBudget = computeMltBudget();
        // if there are seeds, samples have been discarded. They need to be put back
        nbOfChains = pathSeeds.size() > 0 ? std::max((mltBudget + m_config.nMutations -1) / m_config.nMutations, (size_t) 1) : 0;

        // actual mlt budget
        mltBudget = nbOfChains * m_config.nMutations;

        cost += mltBudget;

        if (use_mlt && nbOfChains > 0) {
            m_config.workUnits = std::min(nCores == 1 ? 1 : nCores*4, nbOfChains);

            // Pick seeds proportional to their luminance
            auto seeds = drawSeeds(pathSeeds, nbOfChains);

            updateOutlierSeedsStats(pathSeeds, seeds, outlierCounter, seedCounter);

            // Multiple seeds per work unit
            assert(seeds.size() >= (size_t) m_config.workUnits);
        
            Log(EInfo, "Starting on mlt in iteration %i with %i seeds out of %i candidates. Avg luminance is %f, Std: %f.", 
                        iteration, nbOfChains, pathSeeds.size(), weightedStats.Mean(), std::sqrt(weightedStats.Variance()));


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
        weightedStats.Clear();        
        unweightedStats.Clear();    
        pathStats.Clear();    

        // update the detector for the next iteration.
        detector->update(pathSeeds, nbOfChains, samplesPerPixel);
        pathSeeds.clear();

        if (intermediatePeriod > 0 && iteration != 0 && iteration % intermediatePeriod == 0) {
            writeAvos(THESISLOCATION + "prentjes/test/avos/", std::to_string(iteration));
        }
    }

    if (intermediatePeriod < 0) {
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

    delete detector;
    
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

    RunningStats weightedStatsLocal;
    RunningStats unweightedStatsLocal;
    RunningStats pathStatsLocal;

    SplatList splatList;

    auto invSpp = 1.f/samplesPerPixel;

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
            size_t index = sampler->getSampleIndex();
            
            splatList.clear();
            pathSampler->sampleSplats(offset, splatList);

            auto spec = splatList.splats[0].second;
            auto position = splatList.splats[0].first;
            auto luminance = splatList.luminance;

            // Log(EInfo, "Index: %i    Luminance: %f", index, splatList.splats[0].second.getLuminance());
            
            detector->contribute(position, luminance);

            if (iteration != 0 && j < samplesPerPixel) {
            
                auto weight = detector->calculateWeight(position, luminance, random->nextFloat());

                block->put(position, spec * (1-weight) * invSpp, 1);

                weightedStatsLocal.Push(weight * luminance);
                unweightedStatsLocal.Push(luminance);
                pathStatsLocal.Push((1-weight) * luminance);

                if (weight > 0) {
                    localPathSeeds.emplace_back(Point2((double) position.x / cropSize.x, (double) position.y / cropSize.y), seed, index, luminance, spec);
                } else {
                    inlierMinValue.incrementBase();
                    if (detector->getMin() <= luminance) {
                        ++inlierMinValue;
                    }
                }              
            }
            else if (iteration != 0 && j >= samplesPerPixel) {
                auto weight = detector->calculateWeight(position, luminance, random->nextFloat());
                outlierDomain->put(position, spec * weight * invSpp, 1);
            }
            
            sampler->advance();
        }
    }

    LockGuard lock(seedMutex); // pathSeeds and pathResult are shared by all threads
    pathResult->put(block);

    weightedStats += weightedStatsLocal;
    unweightedStats += unweightedStatsLocal;
    pathStats += pathStatsLocal;

    if (!localPathSeeds.empty()) {
        pathSeeds.insert( pathSeeds.end(), localPathSeeds.begin(), localPathSeeds.end() );
    }
}

// TODO: this is still single threaded
void MyPathTracer::initDetectorMean(const Scene *scene, Sampler *_sampler) {

    iteration = 0;

    auto* sampler = (MyRplSampler*) _sampler;

    auto init_block = [this, sampler, scene](Point2i const& block_offset, Vector2i const& block_size) {

        auto local_sampler = sampler->clone();

        ref<PathSampler> pathSampler = new PathSampler(m_config.technique, scene,
                sampler, sampler, sampler, m_config.maxDepth, m_config.rrDepth,
                false, false, false);

        std::vector<float> samples;
        samples.reserve(4);
        SplatList splatList;

        for (int y = 0; y<block_size.y; ++y) {
            for (int x = 0; x<block_size.x; ++x) {
                auto offset = Point2i(x,y) + block_offset;

                for (size_t j = 0; j<samplesPerPixel; j++) {
                    
                    splatList.clear();
                    pathSampler->sampleSplats(offset, splatList);

                    if (splatList.luminance != 0.f) {
                        samples.push_back(splatList.luminance);    
                    }
                    if (samples.size() >= 4) {
                        break;
                    }
                }
                
                if (samples.size() > 0) {
                    std::sort(samples.begin(), samples.end());
                    float init_value = samples[samples.size()*0.4f];
                    detector->init(offset, init_value);
                    samples.clear();
                } else {
                    detector->init(offset, 0.01f);
                }
            }
        }
    };

    init_block(Point2i(0), cropSize);

    // Vector2i block_size = cropSize/1;
    // std::vector<std::thread> threads;
    // threads.reserve(cropSize.x / block_size.x * cropSize.y / block_size.y);

    // for (int y = 0; y<cropSize.y; y += block_size.y) {
    //     for (int x = 0; x<cropSize.x; x += block_size.x) {
    //         auto actual_block_size = Vector2i{std::min(block_size.x, cropSize.x - x), std::min(block_size.y, cropSize.y - y)};
    //         init_block(Point2i(x,y), actual_block_size);
    //     }
    // }

    // for (auto& thread : threads) {
    //     thread.join();
    // }


    detector->update(0);
    pathSeeds.clear();

    // Set the spp back to the requested value after initialisation
    samplesPerPixel = sampler->getSampleCount();
}

void MyPathTracer::init() {
        m_config.maxDepth = props.getInteger("maxDepth", -1);
        m_config.rrDepth = props.getInteger("rrDepth", 5);
        m_config.technique = props.getBoolean("bidirectional", false) ?
            PathSampler::EBidirectional : PathSampler::EUnidirectional;
        use_mlt = props.getBoolean("mlt", true);

        m_config.mutationSizeLow  = props.getFloat("mutationSizeLow",  1.0f/1024.0f);
        m_config.mutationSizeHigh = props.getFloat("mutationSizeHigh", 1.0f/64.0f);
        Assert(m_config.mutationSizeLow > 0 && m_config.mutationSizeHigh > 0 &&
               m_config.mutationSizeLow < 1 && m_config.mutationSizeHigh < 1 &&
               m_config.mutationSizeLow < m_config.mutationSizeHigh);
        m_config.workUnits = props.getInteger("workUnits", -1);
        /* Stop MLT after X seconds -- useful for equal-time comparisons */
        m_config.timeout = props.getInteger("timeout", 0);
        kappa = props.getFloat("kappa", 1);
        detectorSoftness = props.getFloat("detectorSoftness", 0.01);

        iterations = props.getInteger("iterations", 10);
        outlierDetectorThreshold = props.getFloat("outlierThreshold", 1);
        intermediatePeriod = props.getInteger("intermediatePeriod", 0);
        // -1 means testing for graphs, zero means standard operation
        if (intermediatePeriod > 0 || intermediatePeriod == -1) {
            intermediatePath = props.getString("intermediatePath");
        }

        samplesPerPixel = props.getInteger("samplesFirst", 1);

        detectorType = props.getString("detector");
        assert(detectorType == "bitterli" || detectorType == "mean" || detectorType == "all-in" || detectorType == "all-out");
       
        additionalThresholding = props.getBoolean("additionalThreshold", false);

        // Set chain lenght
        m_config.nMutations = 1000;

        random = new Random();
        seedMutex = new Mutex();
}

bool MyPathTracer::render(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {

    renderSetup(scene, queue, job, sceneResID, sensorResID, samplerResID);


    if (props.getInteger("intermediatePeriod", 0) == -1) {
        assert(cropSize == Vector2i(100));

        for (int loop=0; loop<nPoints; ++loop) {
            float testValue;
            if (exponential) {
                testValue = minValue + std::pow(10, loop * std::log(maxValue-minValue)/std::log(10) /nPoints);
            } else {
                testValue = minValue + loop * (maxValue - minValue)/nPoints;
            }
            
            // Override the property we want to test
            props.removeProperty(testProperty);

            if (testProperty == "iterations") {
                props.setInteger(testProperty, (int) testValue);
            } else {
                props.setFloat(testProperty, testValue);
            }


            

            for (int i=0; i<nSubPoints; ++i) {    
                init();           

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

 void MyPathTracer::writeStatisticsToFile(int nOutliers, int nSeeds, MltStats const& mltStats) const {

     std::string location;
    
    if (intermediatePeriod == 0) {
        location = THESISLOCATION + "prentjes/test/stat_output.txt";
    } else {
        location = intermediatePath + "stat_output.txt";
    }
    std::ofstream myfile(location, std::ios::app);

    if (myfile.is_open())
    {
        if (iteration == 1) {
            myfile << "[\n";
        }

        myfile << "{";

        myfile << "\"iteration\": " << iteration << ", ";
        myfile << "\"nOutliers\": " << nOutliers << ", ";
        myfile << "\"nInliers\": " << samplesTotal - nOutliers << ", ";
        myfile << "\"nInliersMinValue\": " << inlierMinValue.getValue() << ", ";
        myfile << "\"nSeeds\": " << nSeeds << ", ";
        myfile << "\"cost\": " << cost << ", ";
        myfile << "\"r\": " << weightedStats.Mean() / unweightedStats.Mean() << ", ";
        myfile << "\"minValue\": " << detector->getMin() << ", ";
        myfile << "\"nMutations\": " << mltStats.nMutations << ", ";
        myfile << "\"nRejections\": " << mltStats.nRejections << ", ";
        myfile << "\"nRejectionDomain\": " << mltStats.nRejectionDomain << ", ";
        myfile << "\"nRejectionsMinThresh\": " << mltStats.nRejectionDomainMinValue << ", ";
        myfile << "\"varIn\": " << pathStats.Variance() << ", ";
        myfile << "\"varOut\": " << weightedStats.Variance() << ", ";
        myfile << "\"varUnweigthed\": " << unweightedStats.Variance();
        
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
