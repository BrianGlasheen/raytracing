#pragma once

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "math.h"

#include <vector>
#include <string>

using std::vector, std::string;

void load_obj(const string& name, vector<vec3>& positions, vector<vec3>& normals, vector<vec2>& texcoords, vector<uint32>& indices) {
    printf("loading %s\n", name.c_str());

    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> materials;
    string err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, name.c_str())) {
        throw std::runtime_error(err);
    }

    for (const auto& shape : shapes) {
        size_t index_offset = 0;

        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
            size_t fv = shape.mesh.num_face_vertices[f];
            if (fv != 3) continue; // only triangles

            vec3 verts[3];

            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shape.mesh.indices[index_offset + v];

                vec3 position = {
                    attrib.vertices[3 * idx.vertex_index + 0],
                    attrib.vertices[3 * idx.vertex_index + 1],
                    attrib.vertices[3 * idx.vertex_index + 2]
                };
                positions.push_back(position);

                if (!attrib.normals.empty() && idx.normal_index >= 0) {
                    vec3 norm = {
                        attrib.normals[3 * idx.normal_index + 0],
                        attrib.normals[3 * idx.normal_index + 1],
                        attrib.normals[3 * idx.normal_index + 2]
                    };
                    normals.push_back(norm);
                }
                else {
                    normals.push_back(vec3(0.0f));
                }

                if (!attrib.texcoords.empty() && idx.texcoord_index >= 0) {
                    vec2 uv = {
                        attrib.texcoords[2 * idx.texcoord_index + 0],
                        1.0f - attrib.texcoords[2 * idx.texcoord_index + 1] // flip V
                    };
                    texcoords.push_back(uv);
                }
                else {
                    texcoords.push_back(vec2(0.0f));
                }

                indices.push_back((uint32)(positions.size() - 1));
            }

            index_offset += fv;
        }
    }

    printf("loaded %s with:\n", name.c_str());
    printf("positions %d\n", (uint32)positions.size());
    printf("normals %d\n", (uint32)normals.size());
    printf("indices %d\n", (uint32)indices.size());
}
