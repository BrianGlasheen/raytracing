#pragma once

#include "math.h"

#include <algorithm>
#include <vector>
using std::swap, std::vector;

struct bvh_node {
    vec3 aabb_min;
    uint32 left_first;
    vec3 aabb_max;
    uint32 count;
};

void update_node_bounds(vector<bvh_node>& bvh, uint32_t node_index, const vector<vec3>& positions, const vector<uint32_t>& indices) {
    bvh_node& node = bvh[node_index];

    node.aabb_min = vec3(FLT_MAX);
    node.aabb_max = vec3(-FLT_MAX);

    for (uint32_t i = 0; i < node.count; i++) {
        uint32_t base = (node.left_first + i) * 3;
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

void subdivide(vector<bvh_node>& bvh, uint32 nodeIdx, uint32 nodesUsed, vector<vec3>& centroids, const vector<vec3>& positions, vector<uint32>& indices) {
    bvh_node& node = bvh[nodeIdx];

    if (node.count <= 2) return;

    vec3 extent = node.aabb_max - node.aabb_min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float splitPos = node.aabb_min[axis] + extent[axis] * 0.5f;

    int i = node.left_first;
    int j = i + node.count - 1;
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

    int leftCount = i - node.left_first;
    if (leftCount == 0 || leftCount == node.count) return;

    uint32_t leftChildIdx = nodesUsed++;
    uint32_t rightChildIdx = nodesUsed++;

    bvh.emplace_back();
    bvh.emplace_back();
    node = bvh[nodeIdx];

    bvh[leftChildIdx].left_first = node.left_first;
    bvh[leftChildIdx].count = leftCount;
    bvh[rightChildIdx].left_first = i;
    bvh[rightChildIdx].count = node.count - leftCount;

    node.left_first = leftChildIdx;
    node.count = 0;

    update_node_bounds(bvh, leftChildIdx, positions, indices);
    update_node_bounds(bvh, rightChildIdx, positions, indices);

    subdivide(bvh, leftChildIdx, nodesUsed, centroids, positions, indices);
    subdivide(bvh, rightChildIdx, nodesUsed, centroids, positions, indices);
}

void build_bvh(vector<bvh_node>& bvh, const vector<vec3>& positions, vector<uint32>& indices, uint32 base_index, uint32 index_count) {
    assert((index_count % 3) == 0);

    printf("building bvh\n");
    printf("base index: %d, index count: %d\n", base_index, index_count);
    // todo start timer

    // use specific models indices range
    const uint32 num_tris = index_count / 3;
    if (num_tris == 0) return;

    vector<vec3> centroids;
    centroids.reserve(num_tris);
    for (int i = 0; i < num_tris; i++) {
        uint32 i0 = indices[i * 3 + 0]; // add base index
        uint32 i1 = indices[i * 3 + 1]; // add base index
        uint32 i2 = indices[i * 3 + 2]; // add base index
        centroids.push_back((positions[i0] + positions[i1] + positions[i2]) * (1.0f / 3.0f));
    }

    bvh.clear();
    bvh.reserve(num_tris * 2 - 1);
    bvh.emplace_back();

    uint32 nodesUsed = 1;
    uint32 rootNodeIdx = 0;
    bvh_node& root = bvh[rootNodeIdx];
    root.left_first = 0;
    root.count = num_tris;

    update_node_bounds(bvh, rootNodeIdx, positions, indices);
    subdivide(bvh, rootNodeIdx, nodesUsed, centroids, positions, indices);

    printf("built bvh with %d nodes\n", (uint32)bvh.size());
}
