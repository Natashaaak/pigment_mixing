#ifndef SPATULAMESH_H
#define SPATULAMESH_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

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
    size_t getVertexCount() const { return vertex_count; }

private:
    void loadOBJ(const std::string& path);

    GLuint vao = 0;
    GLuint vbo = 0;
    size_t vertex_count = 0;
};

#endif // SPATULAMESH_H