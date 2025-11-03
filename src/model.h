#pragma once

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "bvh.h"
#include "math.h"

#include <vector>
#include <string>

using std::vector, std::string;

struct model {
    mat4 transform;
    mat4 inv_transform;
    size_t mat; // material
    vec3 aabb_min, aabb_max; // same as root nodes aabb
    uint32 base_index, index_count;
    vector<bvh_node> bvh; // blas
};

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

model load_model(const string& name, vector<vec3>& positions, vector<vec3>& normals, vector<vec2>& texcoords, vector<uint32>& indices) {
    // todo check if model loaded
    // if so return copy

    model my_model = { 0 };
    my_model.transform = mat4(1.0f); // todo argument
    my_model.inv_transform = inverse(my_model.transform);
    //my_model.mat = 0; todo

    my_model.base_index = indices.size();
    load_obj(name, positions, normals, texcoords, indices);
    my_model.index_count = indices.size() - my_model.base_index;

    build_bvh(my_model.bvh, positions, indices, my_model.base_index, my_model.index_count);
    //build_bvh(bvh, positions, indices);

    // load data into global buffers
    // build blas over model
    // set model's aabb for tlas

    my_model.aabb_min = my_model.bvh[0].aabb_min;
    my_model.aabb_max = my_model.bvh[0].aabb_max;

    return my_model;
}
