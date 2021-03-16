

#ifndef MITSUBA_MYPATHSEED
#define MITSUBA_MYPATHSEED

#include <mitsuba/core/point.h>
#include <mitsuba/bidir/pathsampler.h>

MTS_NAMESPACE_BEGIN

// struct PositionedPathSeed : public PathSeed {
//     Point2 position;
//     Float pdf;

//     inline PositionedPathSeed(Point2 position, size_t seed, Float luminance, Float pdf =0, int s = 0, int t = 0)
//         : PathSeed(seed, luminance, s, t), position(position) {}

//     inline PositionedPathSeed(Stream *stream) : PathSeed(stream) {
//         Float x = stream->readFloat();
//         Float y = stream->readFloat();
//         position = Point2(x,y);
//     }

//     void serialize(Stream *stream) const {
//         PathSeed::serialize(stream);  
//         stream->writeFloat(position.x);    
//         stream->writeFloat(position.y);
//     }

//     std::string toString() const {
//         std::ostringstream oss;
//         oss << "PathSeed[" << endl
//             << "  seed = " << seed << "," << endl
//             << "  luminance = " << luminance << "," << endl
//             << "  position = " << position.toString() << endl
//             << "  s = " << s << "," << endl
//             << "  t = " << t << endl
//             << "]";
//         return oss.str();
//     }
// };

struct PositionedPathSeed {
    size_t index;       ///< Index into a rewindable random number stream
    uint64_t seed;      ///< Seed of the rng
    Float luminance;    ///< Luminance value of the path (for sanity checks)
    Point2 position;    ///< Position on screen
    Float pdf;          ///< The pdf with which the sample is chosen
    int s;              ///< Number of steps from the luminaire
    int t;              ///< Number of steps from the eye

    inline PositionedPathSeed() { }

    inline PositionedPathSeed(Point2 position, uint64_t seed, size_t index, Float luminance, Float pdf =0, int s = 0, int t = 0)
        : index(index), seed(seed), luminance(luminance), position(position), pdf(pdf), s(s), t(t) {}

    inline PositionedPathSeed(Stream *stream) {
        seed = stream->readSize();
        luminance = stream->readFloat();
        s = stream->readInt();
        t = stream->readInt(); 
        auto x = stream->readFloat();    
        auto y = stream->readFloat();
        position = Point2(x,y);
        pdf = stream->readFloat();
    }

    void serialize(Stream *stream) const {
        stream->writeSize(seed);
        stream->writeFloat(luminance);
        stream->writeInt(s);
        stream->writeInt(t);
        stream->writeFloat(position.x);    
        stream->writeFloat(position.y);
        stream->writeFloat(pdf);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "PathSeed[" << endl
            << "  seed = " << seed << "," << endl
            << "  luminance = " << luminance << "," << endl
            << "  position = " << position.toString() << endl
            << "  pdf = " << pdf << endl
            << "  s = " << s << "," << endl
            << "  t = " << t << endl
            << "]";
        return oss.str();
    }
};


class PositionedSeedWorkUnit : public WorkUnit {
public:
    inline void set(const WorkUnit *wu) {
        m_seeds = static_cast<const PositionedSeedWorkUnit*>(wu)->m_seeds;
        m_timeout = static_cast<const PositionedSeedWorkUnit*>(wu)->m_timeout;
    }

    inline const std::vector<PositionedPathSeed> &getSeeds() const {
        return m_seeds;
    }

    inline void setSeeds(const std::vector<PositionedPathSeed> &seeds) {
        m_seeds = seeds;
    }

    inline int getTimeout() const {
        return m_timeout;
    }

    inline void setTimeout(int timeout) {
        m_timeout = timeout;
    }

    //TODO
    // inline void load(Stream *stream) {
    //     m_seeds = PositionedPathSeed(stream);
    //     m_timeout = stream->readInt();
    // }

    // inline void save(Stream *stream) const {
    //     m_seed.serialize(stream);
    //     stream->writeInt(m_timeout);
    // }

    inline std::string toString() const {
        return "SeedWorkUnit[]";
    }

    MTS_DECLARE_CLASS()
private:
    std::vector<PositionedPathSeed> m_seeds;
    int m_timeout;
};

MTS_NAMESPACE_END

#endif

