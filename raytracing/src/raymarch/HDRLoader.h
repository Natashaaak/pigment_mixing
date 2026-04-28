#ifndef HDRLOADER_H
#define HDRLOADER_H

#include <glad/glad.h>
#include <string>

class HDRLoader {
public:
    /**
     * Načte HDR obrázek ze zadané cesty, převede ho na Cubemap texturu a vygeneruje Irradiance mapu pro IBL.
     */
    static void loadHDRCubemap(const std::string& path, GLuint& envMap, GLuint& irradianceMap);
};

#endif //HDRLOADER_H