#include <mitsuba/mitsuba.h>

#ifndef MYPSSMLTCONFIG
#define MYPSSMLTCONFIG

MTS_NAMESPACE_BEGIN

/**
 * \brief Stores all configuration parameters used by
 * the MLT rendering implementation
 */
struct MYPSSMLTConfiguration {
    PathSampler::ETechnique technique;
    int maxDepth;
    int rrDepth;
    int workUnits;
    size_t nMutations;
    Float mutationSizeLow;
    Float mutationSizeHigh;
    size_t timeout;

    inline MYPSSMLTConfiguration() { }

    void dump() const {
        SLog(EDebug, "PSSMLT configuration:");
        SLog(EDebug, "   Maximum path depth          : %i", maxDepth);
        SLog(EDebug, "   Bidirectional path tracing  : %s",
            (technique == PathSampler::EBidirectional) ? "yes" : "no");
        SLog(EDebug, "   Russian roulette depth      : %i", rrDepth);
        SLog(EDebug, "   Mutation size               : [%f, %f]",
            mutationSizeLow, mutationSizeHigh);
        SLog(EDebug, "   Total number of work units  : %i", workUnits);
        SLog(EDebug, "   Mutations per work unit     : " SIZE_T_FMT, nMutations);
    }

    inline MYPSSMLTConfiguration(Stream *stream) {
        technique = (PathSampler::ETechnique) stream->readUInt();
        maxDepth = stream->readInt();
        rrDepth = stream->readInt();    
        workUnits = stream->readInt();    
        nMutations = stream->readSize();       
        mutationSizeLow = stream->readFloat();
        mutationSizeHigh = stream->readFloat();
        timeout = stream->readSize();
    }

    inline void serialize(Stream *stream) const {
        stream->writeUInt((uint32_t) technique);
        stream->writeInt(maxDepth);     
        stream->writeInt(rrDepth);     
        stream->writeInt(workUnits);      
        stream->writeSize(nMutations);     
        stream->writeFloat(mutationSizeLow);
        stream->writeFloat(mutationSizeHigh);
        stream->writeSize(timeout);
    }
};

MTS_NAMESPACE_END

#endif