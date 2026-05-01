#ifndef SPATULAMESH_H
#define SPATULAMESH_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

class Shader;
class Camera;

struct MaterialGroup {
    int material_id;
    std::string material_name;
    unsigned int offset;
    unsigned int count;
};

struct SpatulaMaterial {
    glm::vec3 albedo;
    float metallic;
    float roughness;
};

class SpatulaMesh {
public:
    SpatulaMesh(const std::string& path);
    ~SpatulaMesh();

    GLuint getVAO() const { return vao; }
    GLuint getVBO() const { return vbo; }
    const std::vector<MaterialGroup>& getGroups() const { return groups; }

    void render(Shader* shader, const glm::mat4& invSpatulaTransform, Camera* camera, bool fullRender, GLuint hdrTexture, GLuint irradianceTexture, GLuint brdfLUTTexture, const SpatulaMaterial& woodMat, const SpatulaMaterial& metalMat) const;

private:
    void loadOBJ(const std::string& path);

    GLuint vao = 0;
    GLuint vbo = 0;
    std::vector<MaterialGroup> groups;
};

#endif // SPATULAMESH_H