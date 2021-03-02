
#ifndef MITSUBA_WRITE_BITMAP
#define MITSUBA_WRITE_BITMAP

#include <string>

#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/render/film.h>


MTS_NAMESPACE_BEGIN

class BitmapWriter {

public:
    enum EOutType { EHDR, ELDR };

    static void writeBitmap(Bitmap const* bitmap, EOutType type, std::string const& path) {
        Properties props;
        switch (type) {
            case EHDR : props.setPluginName("hdrfilm"); break;
            case ELDR : props.setPluginName("ldrfilm"); break;
        }
        auto size = bitmap->getSize();
        props.setInteger("width", size.x);
        props.setInteger("height", size.y);
        props.setInteger("cropWidth", size.x);
        props.setInteger("cropHeight", size.y);
        props.setBool("banner", false);


        ref<Film> film = static_cast<Film *> (PluginManager::getInstance()->createObject(MTS_CLASS(Film), props));

        film->setBitmap(bitmap);
        film->setDestinationFile(path, 0);

        film->develop(nullptr, 0);
    }
};

MTS_NAMESPACE_END

#endif