
#include <mitsuba/core/statistics.h>
#include <mitsuba/core/sfcurve.h>
#include <mitsuba/render/renderproc.h>
#include <mitsuba/render/rectwu.h>

#include "myrenderproc.h"
#include "myPathTracer.h"

MTS_NAMESPACE_BEGIN


MyBlockRenderer::MyBlockRenderer(Bitmap::EPixelFormat pixelFormat, int channelCount, int blockSize,
    int borderSize, bool warnInvalid) : m_pixelFormat(pixelFormat),
    m_channelCount(channelCount), m_blockSize(blockSize),
    m_borderSize(borderSize), m_warnInvalid(warnInvalid) { }

MyBlockRenderer::MyBlockRenderer(Stream *stream, InstanceManager *manager) {
    m_pixelFormat = (Bitmap::EPixelFormat) stream->readInt();
    m_channelCount = stream->readInt();
    m_blockSize = stream->readInt();
    m_borderSize = stream->readInt();
    m_warnInvalid = stream->readBool();
}

ref<WorkUnit> MyBlockRenderer::createWorkUnit() const {
    return new RectangularWorkUnit();
}

ref<WorkResult> MyBlockRenderer::createWorkResult() const {
    return new ImageBlock(m_pixelFormat,
        Vector2i(m_blockSize),
        m_sensor->getFilm()->getReconstructionFilter(),
        m_channelCount, m_warnInvalid);
}

void MyBlockRenderer::prepare() {
    Scene *scene = static_cast<Scene *>(getResource("scene"));
    m_scene = new Scene(scene);
    m_sampler = static_cast<Sampler *>(getResource("sampler"));
    m_sensor = static_cast<Sensor *>(getResource("sensor"));
    m_integrator = static_cast<MyPathTracer*>(getResource("integrator"));
    m_scene->removeSensor(scene->getSensor());
    m_scene->addSensor(m_sensor);
    m_scene->setSensor(m_sensor);
    m_scene->setSampler(m_sampler);
    m_scene->setIntegrator(m_integrator);
    m_integrator->wakeup(m_scene, m_resources);
    m_scene->wakeup(m_scene, m_resources);
    m_scene->initializeBidirectional();
}

void MyBlockRenderer::process(const WorkUnit *workUnit, WorkResult *workResult,
    const bool &stop) {
    const RectangularWorkUnit *rect = static_cast<const RectangularWorkUnit*>(workUnit);
    ImageBlock *block = static_cast<ImageBlock *>(workResult);

#ifdef MTS_DEBUG_FP
    enableFPExceptions();
#endif

    block->setOffset(rect->getOffset());
    block->setSize(rect->getSize());
    m_hilbertCurve.initialize(TVector2<uint8_t>(rect->getSize()));
    m_integrator->renderBlock(m_scene, m_sensor, m_sampler,
        block, stop, m_hilbertCurve.getPoints());

#ifdef MTS_DEBUG_FP
    disableFPExceptions();
#endif
}

void MyBlockRenderer::serialize(Stream *stream, InstanceManager *manager) const {
    stream->writeInt(m_pixelFormat);
    stream->writeInt(m_channelCount);
    stream->writeInt(m_blockSize);
    stream->writeInt(m_borderSize);
    stream->writeBool(m_warnInvalid);
}

ref<WorkProcessor> MyBlockRenderer::clone() const {
    return new MyBlockRenderer(m_pixelFormat, m_channelCount,
        m_blockSize, m_borderSize, m_warnInvalid);
}



BlockedRenderProcess::BlockedRenderProcess(const RenderJob *parent, RenderQueue *queue,
        int blockSize) : m_queue(queue), m_parent(parent), m_resultCount(0), m_progress(NULL) {
    m_blockSize = blockSize;
    m_resultMutex = new Mutex();
    m_pixelFormat = Bitmap::ESpectrumAlphaWeight;
    m_channelCount = -1;
    m_warnInvalid = true;
}

BlockedRenderProcess::~BlockedRenderProcess() {
    if (m_progress)
        delete m_progress;
}

void BlockedRenderProcess::setPixelFormat(Bitmap::EPixelFormat pixelFormat, int channelCount, bool warnInvalid) {
    m_pixelFormat = pixelFormat;
    m_channelCount = channelCount;
    m_warnInvalid = warnInvalid;
}

ref<WorkProcessor> BlockedRenderProcess::createWorkProcessor() const {
    return new MyBlockRenderer(m_pixelFormat, m_channelCount,
            m_blockSize, m_borderSize, m_warnInvalid);
}

void BlockedRenderProcess::processResult(const WorkResult *result, bool cancelled) {
    const ImageBlock *block = static_cast<const ImageBlock *>(result);
    UniqueLock lock(m_resultMutex);
    m_film->put(block);
    m_progress->update(++m_resultCount);
    lock.unlock();
    m_queue->signalWorkEnd(m_parent, block, cancelled);
}

ParallelProcess::EStatus BlockedRenderProcess::generateWork(WorkUnit *unit, int worker) {
    EStatus status = BlockedImageProcess::generateWork(unit, worker);
    if (status == ESuccess)
        m_queue->signalWorkBegin(m_parent, static_cast<RectangularWorkUnit *>(unit), worker);
    return status;
}

void BlockedRenderProcess::bindResource(const std::string &name, int id) {
    if (name == "sensor") {
        m_film = static_cast<Sensor *>(Scheduler::getInstance()->getResource(id))->getFilm();
        m_borderSize = m_film->getReconstructionFilter()->getBorderSize();

        Point2i offset = Point2i(0, 0);
        Vector2i size = m_film->getCropSize();

        if (m_film->hasHighQualityEdges()) {
            offset.x -= m_borderSize;
            offset.y -= m_borderSize;
            size.x += 2 * m_borderSize;
            size.y += 2 * m_borderSize;
        }

        if (m_blockSize < m_borderSize)
            Log(EError, "The block size must be larger than the image reconstruction filter radius!");

        BlockedImageProcess::init(offset, size, m_blockSize);
        if (m_progress)
            delete m_progress;
        m_progress = new ProgressReporter("Rendering", m_numBlocksTotal, m_parent);
    }
    BlockedImageProcess::bindResource(name, id);
}

MTS_NAMESPACE_END
