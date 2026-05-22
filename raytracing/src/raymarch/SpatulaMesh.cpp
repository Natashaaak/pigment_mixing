#include "SpatulaMesh.h"
#include <spdlog/spdlog.h>
#include "../Shader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "../../external/tiny_obj_loader.h"

SpatulaMesh::SpatulaMesh(const std::string& path) {
    loadOBJ(path);
}

SpatulaMesh::~SpatulaMesh() {
    if (vao) glDeleteVertexArrays(1, &vao);
    if (vbo) glDeleteBuffers(1, &vbo);
}

void SpatulaMesh::loadOBJ(const std::string& path) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    // Get the directory from the specified path so tinyobjloader can find a possible .mtl file
    std::string base_dir = "";
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        base_dir = path.substr(0, pos + 1);
    }

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str(), base_dir.c_str());

    if (!warn.empty()) {
        spdlog::warn("TinyObjReader: {}", warn);
    }
    if (!err.empty()) {
        spdlog::error("TinyObjReader: {}", err);
    }
    if (!ret) {
        spdlog::error("Failed to load OBJ: {}", path);
        return;
    }

    struct Vertex {
        glm::vec3 pos;
        glm::vec3 normal;
        int material_id; // 0 = metal, 1 = wood
    };

    std::vector<Vertex> finalVertices;
    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            if (fv != 3) { // We only support triangles
                index_offset += fv;
                continue;
            }

            int material_id = shapes[s].mesh.material_ids[f];
            int mat_idx_for_shader = 0; // Default to metal
            if (material_id >= 0 && material_id < materials.size()) {
                const std::string& mat_name = materials[material_id].name;
                if (mat_name.find("wood") != std::string::npos || 
                    mat_name.find("Wood") != std::string::npos || 
                    mat_name.find("WOOD") != std::string::npos) {
                    mat_idx_for_shader = 1; // Wood
                }
            }
            // Fallback for models without clear material names, assuming second material is wood.
            else if (material_id == 1) {
                mat_idx_for_shader = 1;
            }

            for (size_t v = 0; v < 3; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                Vertex vertex;
                
                vertex.pos = {
                    attrib.vertices[3 * idx.vertex_index + 0],
                    attrib.vertices[3 * idx.vertex_index + 1],
                    attrib.vertices[3 * idx.vertex_index + 2]
                };

                if (idx.normal_index >= 0) {
                    vertex.normal = {
                        attrib.normals[3 * idx.normal_index + 0],
                        attrib.normals[3 * idx.normal_index + 1],
                        attrib.normals[3 * idx.normal_index + 2]
                    };
                } else {
                    vertex.normal = {0.0f, 0.0f, 0.0f};
                }

                vertex.material_id = mat_idx_for_shader;
                finalVertices.push_back(vertex);
            }
            index_offset += 3;
        }
    }

    vertex_count = finalVertices.size();

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertex_count * sizeof(Vertex), finalVertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    
    glEnableVertexAttribArray(2);
    glVertexAttribIPointer(2, 1, GL_INT, sizeof(Vertex), (void*)offsetof(Vertex, material_id));

    glBindVertexArray(0);

    std::cout << "Loaded OBJ: " << path << " with " << vertex_count << " vertices." << std::endl;
}