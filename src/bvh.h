#pragma once

#include "math.h"

#include <algorithm>
using std::swap;

struct bvh_node {
    vec3 aabb_min, aabb_max;
    uint32_t left, right;
    uint32 firstPrim, primCount;
};

uint32_t nodesUsed = 1;
vector<vec3> centroids;

void update_node_bounds(vector<bvh_node>& bvh, uint32_t node_index, const vector<vec3>& positions, const vector<uint32_t>& indices) {
    bvh_node& node = bvh[node_index];

    node.aabb_min = vec3(FLT_MAX);
    node.aabb_max = vec3(-FLT_MAX);

    for (uint32_t i = 0; i < node.primCount; i++) {
        uint32_t base = (node.firstPrim + i) * 3;
        uint32_t idx0 = indices[base + 0];
        uint32_t idx1 = indices[base + 1];
        uint32_t idx2 = indices[base + 2];

        vec3 v0 = positions[idx0];
        vec3 v1 = positions[idx1];
        vec3 v2 = positions[idx2];

        node.aabb_min = min(node.aabb_min, min(v0, min(v1, v2)));
        node.aabb_max = max(node.aabb_max, max(v0, max(v1, v2)));
    }
}

void subdivide(vector<bvh_node>& bvh, uint32_t nodeIdx, const vector<vec3>& positions, vector<uint32_t>& indices) {
    bvh_node& node = bvh[nodeIdx];

    if (node.primCount <= 2) return;

    vec3 extent = node.aabb_max - node.aabb_min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float splitPos = node.aabb_min[axis] + extent[axis] * 0.5f;

    int i = node.firstPrim;
    int j = i + node.primCount - 1;
    while (i <= j) {
        if (centroids[i][axis] < splitPos)
            i++;
        else {
            swap(centroids[i], centroids[j]);
            swap(indices[i * 3 + 0], indices[j * 3 + 0]);
            swap(indices[i * 3 + 1], indices[j * 3 + 1]);
            swap(indices[i * 3 + 2], indices[j * 3 + 2]);
            j--;
        }
    }

    int leftCount = i - node.firstPrim;
    if (leftCount == 0 || leftCount == node.primCount) return;

    uint32_t leftChildIdx = nodesUsed++;
    uint32_t rightChildIdx = nodesUsed++;

    bvh.emplace_back();
    bvh.emplace_back();
    node = bvh[nodeIdx];

    bvh[leftChildIdx].firstPrim = node.firstPrim;
    bvh[leftChildIdx].primCount = leftCount;
    bvh[rightChildIdx].firstPrim = i;
    bvh[rightChildIdx].primCount = node.primCount - leftCount;

    node.left = leftChildIdx;
    node.right = rightChildIdx;
    node.primCount = 0;

    update_node_bounds(bvh, leftChildIdx, positions, indices);
    update_node_bounds(bvh, rightChildIdx, positions, indices);

    subdivide(bvh, leftChildIdx, positions, indices);
    subdivide(bvh, rightChildIdx, positions, indices);
}

void build_bvh(vector<bvh_node>& bvh, const vector<vec3>& positions, vector<uint32_t>& indices) {
    printf("building bvh\n");

    assert((indices.size() % 3) == 0);

    const uint32_t num_tris = (uint32)(indices.size() / 3);
    if (num_tris == 0) return;

    centroids.reserve(num_tris);
    for (int i = 0; i < num_tris; i++) {
        uint32_t i0 = indices[i * 3 + 0];
        uint32_t i1 = indices[i * 3 + 1];
        uint32_t i2 = indices[i * 3 + 2];
        centroids.push_back((positions[i0] + positions[i1] + positions[i2]) * (1.0f / 3.0f));
    }

    bvh.clear();
    bvh.reserve(num_tris * 2 - 1);
    bvh.emplace_back();

    nodesUsed = 1;
    uint32_t rootNodeIdx = 0;
    bvh_node& root = bvh[rootNodeIdx];
    root.firstPrim = 0;
    root.primCount = num_tris;

    update_node_bounds(bvh, rootNodeIdx, positions, indices);
    subdivide(bvh, rootNodeIdx, positions, indices);

    printf("built bvh with %d nodes\n", (uint32)bvh.size());
}
