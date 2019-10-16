#pragma once

#include <glm/glm.hpp>

#include <memory>
#include <vector>
#include <type_traits>

using Vertex = glm::vec3;
using Normal = glm::vec3;
using Index = glm::uvec3;

class Mesh
{
public:

    Mesh() {}

    template<typename T>
    void addVertex(T&& vert);

    template<typename T>
    void addNormal(T&& norm);

    template<typename T>
    void addIndex(T&& idx);

    auto getVertexAt(size_t index) { return vertices[index]; }
    auto getNormalAt(size_t index) { return normals[index]; }
    auto getIndexAt(size_t index) { return indices[index]; }

private:

    std::vector<Vertex> vertices;
    std::vector<Normal> normals;
    std::vector<Index>  indices;
};

template<typename T>
void Mesh::addVertex(T&& vert)
{
    //static_assert(std::is_same_v<decltype(vert), Vertex>);

    vertices.emplace_back(vert);
}

template<typename T>
void Mesh::addNormal(T&& norm)
{
    //static_assert(std::is_same_v<decltype(norm), Normal>);

    normals.emplace_back(norm);
}

template<typename T>
void Mesh::addIndex(T&& idx)
{
    //static_assert(std::is_same_v<decltype(idx), Index>);

    indices.emplace_back(idx);
}