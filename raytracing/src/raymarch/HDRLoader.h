#ifndef HDRLOADER_H
#define HDRLOADER_H

#include <glad/glad.h>
#include <string>

class HDRLoader {
public:
    /**
     * Loads an HDR image from the specified path, converts it to a Cubemap texture, and generates an Irradiance map for IBL.
     */
    static void loadHDRCubemap(const std::string& path, GLuint& envMap, GLuint& irradianceMap);
};

#endif //HDRLOADER_H