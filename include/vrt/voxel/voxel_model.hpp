#pragma once
#include <vector>
#include <string>
#include <optional>
#include <glm/glm.hpp>
#include <vrt/accel/dag.hpp>

namespace vrt
{
    class VoxelModel
    {
    public:
        int size = 512;
        std::vector<Dag::Voxel> grid;

        bool load_obj(const std::string& filepath, int expected_size = 512);

        std::optional<Dag::Voxel> sample(glm::vec3 pos) const;
    };
}
