

#ifndef MITSUBA_MYPATHSEED
#define MITSUBA_MYPATHSEED

#include <mitsuba/core/point.h>
#include <mitsuba/bidir/pathsampler.h>

MTS_NAMESPACE_BEGIN

struct PositionedPathSeed : public PathSeed {
    Point2i position;

    inline PositionedPathSeed(Point2i position, size_t sampleIndex, Float luminance, int s = 0, int t = 0)
        : PathSeed(sampleIndex, luminance, s, t), position(position) {}

    inline PositionedPathSeed(Stream *stream) : PathSeed(stream) {
        Float x = stream->readFloat();
        Float y = stream->readFloat();
        position = Point2i(x,y);
    }

    void serialize(Stream *stream) const {
        PathSeed::serialize(stream);  
        stream->writeFloat(position.x);    
        stream->writeFloat(position.y);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "PathSeed[" << endl
            << "  sampleIndex = " << sampleIndex << "," << endl
            << "  luminance = " << luminance << "," << endl
            << "  position = " << position.toString() << endl
            << "  s = " << s << "," << endl
            << "  t = " << t << endl
            << "]";
        return oss.str();
    }
};


class PositionedSeedWorkUnit : public WorkUnit {
public:
    inline void set(const WorkUnit *wu) {
        m_seed = static_cast<const PositionedSeedWorkUnit*>(wu)->m_seed;
        m_timeout = static_cast<const PositionedSeedWorkUnit*>(wu)->m_timeout;
    }

    inline const PositionedPathSeed &getSeed() const {
        return m_seed;
    }

    inline void setSeed(const PositionedPathSeed &seed) {
        m_seed = seed;
    }

    inline int getTimeout() const {
        return m_timeout;
    }

    inline void setTimeout(int timeout) {
        m_timeout = timeout;
    }

    inline void load(Stream *stream) {
        m_seed = PositionedPathSeed(stream);
        m_timeout = stream->readInt();
    }

    inline void save(Stream *stream) const {
        m_seed.serialize(stream);
        stream->writeInt(m_timeout);
    }

    inline std::string toString() const {
        return "SeedWorkUnit[]";
    }

    MTS_DECLARE_CLASS()
private:
    PositionedPathSeed m_seed;
    int m_timeout;
};

MTS_NAMESPACE_END

#endif

