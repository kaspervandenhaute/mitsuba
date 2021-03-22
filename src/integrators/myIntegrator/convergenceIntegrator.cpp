
#include "convergenceIntegrator.h"

MTS_NAMESPACE_BEGIN

ConvergenceIntegrator::ConvergenceIntegrator(const Properties &props) : props(props) {

}

bool render(Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) {
            for (int i=0; i<1000; i++) {
                auto integrator = static_cast<SamplingIntegrator *> (PluginManager::getInstance()->createObject(MTS_CLASS(MyPathTracer), props));
                integrator->render(scene, queu, job, sceneResID, sensorResID, samplerResID);
                
            }
        }


MTS_NAMESPACE_END


MTS_IMPLEMENT_CLASS(ConvergenceIntegrator, false, Integrator);
MTS_EXPORT_PLUGIN(ConvergenceIntegrator, "convergenceIntegrator");

