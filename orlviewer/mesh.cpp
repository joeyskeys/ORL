#include "mesh.h"

#include <type_traits>

void Mesh::addVertex(auto&& vert)
{
    static_assert(std::is_same<decltype(vert), Vertex>::value);

    vertices.emplace_back(vert);
}

void Mesh::addNormal(auto&& norm)
{
    static_assert(std::is_same<decltype(norm), Normal>::value);

    normals.emplace_back(norm);
}

void Mesh::addIndex(auto&& idx)
{
    static_assert(std::is_same<decltype(idx), Index>::value);

    indices.emplace_back(idx);
}
