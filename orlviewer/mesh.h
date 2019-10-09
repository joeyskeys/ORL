#include <glm/glm.hpp>

#include <memory>
#include <vector>

using Vertex = glm::vec3;
using Normal = glm::vec3;
using Index = glm::uvec3;

class Mesh
{
public:

    Mesh() {}

    void addVertex(auto&& vert);
    void addNormal(auto&& norm);
    void addIndex(auto&& idx);

private:

    std::vector<Vertex> vertices;
    std::vector<Normal> normals;
    std::vector<Index>  indices;
};
