#pragma once
#include <vrt/core/types.hpp>
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
#include <array>
#include <span>
#include <optional>
#include <print>
#include <vrt/math/ray.hpp>
#include <functional>
#include <filesystem>

namespace vrt
{
    class Dag
    {
    public:
        union Voxel
        {
            u32 rgbe;
            struct
            {
                u8 e;
                u8 b;
                u8 g;
                u8 r;
            };

            bool operator==(const Voxel& other) const
            {
                return rgbe == other.rgbe;
            };
        };

        union Node
        {
            u64 raw;
            struct
            {
                u32 index;
                union
                {
                    u32 descriptor;
                    Voxel voxel;
                };
            };

            bool operator==(const Node& other) const
            {
                return raw == other.raw;
            }

            const bool is_leaf() const { return index & (1u << 31); }

            void set_child_offset(int octant, u32 offset)
            {
                u32 safe_offset = (offset & 0x7) | 0x8;

                descriptor &= ~(0xF << (octant * 4));
                descriptor |= (safe_offset << (octant * 4));
            }
        };

        struct Hit
        {
            float t;
            glm::vec3 normal;
            Voxel voxel;
        };

        const std::vector<Node>& nodes() { return nodes_; }
        const std::vector<Voxel>& leaves() { return leaves_; }

        Node build(u8 depth, const std::filesystem::path& filepath);

        Hit intersect(const Ray& ray, u8 depth, const Node& root) const noexcept;

        void debug()
        {
            std::size_t node_bytes = (nodes_.size() * sizeof(Node));
            std::size_t leaf_bytes = (leaves_.size() * sizeof(Voxel));
            std::println("=== DAG debug ====");
            std::println("Nodes:   {:<10} | {:>10} B", nodes_.size(), node_bytes);
            std::println("Voxels:  {:<10} | {:>10} B", leaves_.size(), leaf_bytes);

            std::println("Used memory for nodes and voxels {} MB", (node_bytes + leaf_bytes) / 1024.0f / 1024.0f);
        }
    private:
        std::vector<Node>  nodes_;
        std::vector<Voxel> leaves_;

        std::unordered_multimap<u64, u32> node_indices_;
        std::unordered_multimap<u64, u32> leaf_indices_;
    };
}