#include "SpatulaMesh.h"
#include <spdlog/spdlog.h>
#include <map>
#include "../Camera.h"
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

    // Získání složky ze zadané cesty, aby tinyobjloader našel případný .mtl soubor
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
        glm::vec2 texcoord;
    };

    // Mapa pro roztřídění vrcholů podle material_name
    std::map<std::string, std::vector<Vertex>> materialToVertices;

    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            if (fv != 3) { // Podporujeme pouze trojúhelníky
                index_offset += fv;
                continue;
            }

            int material_id = shapes[s].mesh.material_ids[f];
            std::string mat_name = "default";
            if (material_id >= 0 && material_id < materials.size()) {
                mat_name = materials[material_id].name;
            } else if (material_id >= 0) {
                mat_name = "material_" + std::to_string(material_id);
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

                if (idx.texcoord_index >= 0) {
                    vertex.texcoord = {
                        attrib.texcoords[2 * idx.texcoord_index + 0],
                        attrib.texcoords[2 * idx.texcoord_index + 1]
                    };
                } else {
                    vertex.texcoord = {0.0f, 0.0f};
                }

                materialToVertices[mat_name].push_back(vertex);
            }
            index_offset += 3;
        }
    }

    std::vector<Vertex> finalVertices;
    groups.clear();

    // Sloučení rozdělených vrcholů zpět do jednoho bufferu a vytvoření metadat pro vykreslování
    for (auto& pair : materialToVertices) {
        MaterialGroup group;
        group.material_id = -1;
        group.material_name = pair.first;
        group.offset = finalVertices.size();
        group.count = pair.second.size();
        groups.push_back(group);

        finalVertices.insert(finalVertices.end(), pair.second.begin(), pair.second.end());
    }

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, finalVertices.size() * sizeof(Vertex), finalVertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texcoord));

    glBindVertexArray(0);

    spdlog::info("Loaded OBJ: {} with {} vertices across {} material groups.", path, finalVertices.size(), groups.size());
}

void SpatulaMesh::render(Shader* shader, const glm::mat4& invSpatulaTransform, Camera* camera, bool fullRender, GLuint hdrTexture, GLuint irradianceTexture, GLuint brdfLUTTexture, const SpatulaMaterial& woodMat, const SpatulaMaterial& metalMat, const glm::vec3 lightDirs[2], const glm::vec3 lightColors[2]) const {
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS); // Vykreslí se jen to, co je před fluidem
    
    shader->use();
    
    shader->setUniform("invSpatulaTransform", invSpatulaTransform);
    shader->setUniform("view", camera->getView());
    shader->setUniform("projection", camera->getProj());
    shader->setUniform("camPos", camera->cameraPos);
    shader->setUniform("fullRender", fullRender);
    shader->setUniform("lightDirs[0]", lightDirs[0]);
    shader->setUniform("lightDirs[1]", lightDirs[1]);
    shader->setUniform("lightColors[0]", lightColors[0]);
    shader->setUniform("lightColors[1]", lightColors[1]);
    
    if (hdrTexture) { glActiveTexture(GL_TEXTURE5); glBindTexture(GL_TEXTURE_CUBE_MAP, hdrTexture); shader->setUniform("hdrMap", 5); }
    if (irradianceTexture) { glActiveTexture(GL_TEXTURE6); glBindTexture(GL_TEXTURE_CUBE_MAP, irradianceTexture); shader->setUniform("irradianceMap", 6); }
    if (brdfLUTTexture) { glActiveTexture(GL_TEXTURE7); glBindTexture(GL_TEXTURE_2D, brdfLUTTexture); shader->setUniform("brdfLUT", 7); }

    glBindVertexArray(vao);
    int groupIndex = 0;
    for (const auto& group : groups) {
        bool isWood = false;
        if (group.material_name.find("wood") != std::string::npos || 
            group.material_name.find("Wood") != std::string::npos || 
            group.material_name.find("WOOD") != std::string::npos) {
            isWood = true;
        } else if (group.material_name.find("metal") == std::string::npos && 
                   group.material_name.find("Metal") == std::string::npos && 
                   groupIndex == 1) {
            isWood = true; // Fallback
        }
        
        if (isWood) {
            shader->setUniform("spatulaMat.albedo", woodMat.albedo);
            shader->setUniform("spatulaMat.metallic", woodMat.metallic);
            shader->setUniform("spatulaMat.roughness", woodMat.roughness);
        } else {
            shader->setUniform("spatulaMat.albedo", metalMat.albedo);
            shader->setUniform("spatulaMat.metallic", metalMat.metallic);
            shader->setUniform("spatulaMat.roughness", metalMat.roughness);
        }
        glDrawArrays(GL_TRIANGLES, group.offset, group.count);
        groupIndex++;
    }
    glBindVertexArray(0);
}