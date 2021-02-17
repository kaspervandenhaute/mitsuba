
#include <mitsuba/core/statistics.h>
#include <mitsuba/core/sfcurve.h>
#include <mitsuba/render/renderproc.h>
#include <mitsuba/render/rectwu.h>


#ifndef MY_RENDER_PROC
#define MY_RENDER_PROC

MTS_NAMESPACE_BEGIN

class MyPathTracer;

class MyBlockRenderer : public WorkProcessor {
public:
    MyBlockRenderer(Bitmap::EPixelFormat pixelFormat, int channelCount, int blockSize,
        int borderSize, bool warnInvalid);

    MyBlockRenderer(Stream *stream, InstanceManager *manager);

    ref<WorkResult> createWorkResult() const;

    ref<WorkUnit> createWorkUnit() const;

    void prepare();

    void process(const WorkUnit *workUnit, WorkResult *workResult,
        const bool &stop);

    void serialize(Stream *stream, InstanceManager *manager) const;

    ref<WorkProcessor> clone() const;

protected:
    virtual ~MyBlockRenderer() { }
private:
    ref<Scene> m_scene;
    ref<Sensor> m_sensor;
    ref<Sampler> m_sampler;
    ref<MyPathTracer> m_integrator;
    Bitmap::EPixelFormat m_pixelFormat;
    int m_channelCount;
    int m_blockSize;
    int m_borderSize;
    bool m_warnInvalid;
    HilbertCurve2D<uint8_t> m_hilbertCurve;
    
};

MTS_NAMESPACE_END

#endif
