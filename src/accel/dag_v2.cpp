#include <vrt/accel/dag_v2.hpp>
#include <iostream>
#include <unordered_map>
#include <fstream>
#include <algorithm>

namespace vrt
{
    namespace v2
    {

        // =========================================================================
        // STRUKTURY POMOCNICZE
        // =========================================================================

        struct TempNode
        {
            bool valid[8] = { false };
            vrt::u32 children[8] = { 0 };

            void add(vrt::u8 octant, vrt::u32 child_idx)
            {
                valid[octant] = true;
                children[octant] = child_idx;
            }

            bool is_empty() const
            {
                for (int i = 0; i < 8; ++i) if (valid[i]) return false;
                return true;
            }

            void clear()
            {
                for (int i = 0; i < 8; ++i) valid[i] = false;
            }

            vrt::u32 emit(std::vector<vrt::u32>& level_buffer)
            {
                vrt::u32 descriptor = 0;
                std::vector<vrt::u32> unique_children;

                for (int i = 0; i < 8; ++i)
                {
                    if (!valid[i]) continue;

                    vrt::u32 offset = 0;
                    bool found = false;
                    for (size_t j = 0; j < unique_children.size(); ++j)
                    {
                        if (unique_children[j] == children[i])
                        {
                            offset = j;
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        offset = unique_children.size();
                        unique_children.push_back(children[i]);
                    }

                    vrt::u32 chunk = 0b1000 | (offset & 0b0111);
                    descriptor |= (chunk << (i * 4));
                }

                vrt::u32 start_idx = level_buffer.size();
                level_buffer.push_back(descriptor);
                for (vrt::u32 c : unique_children)
                {
                    level_buffer.push_back(c);
                }
                return start_idx;
            }
        };

        struct StackFrame
        {
            vrt::u32 node_idx;
            float tx, ty, tz;
            float t_exit;
            vrt::u8 oct_idx;
        };

        // =========================================================================
        // BUILDER (Kompresja SVO -> SVDAG)
        // =========================================================================

        vrt::u32 Dag::build(u8 depth, const std::filesystem::path& filepath)
        {
            if (!std::filesystem::exists(filepath))
            {
                throw std::runtime_error("Error: vrt::v2::Dag::build filepath doesn't exist.");
            }

            std::ifstream file(filepath, std::ios::binary);

            nodes_.clear();
            geometry_leaves_.clear();

            std::unordered_map<u8, u32> leaf_dict;

            auto get_or_add_leaf = [&](TempNode& leaf) -> u32
                {
                    u8 mask = 0;
                    for (int i = 0; i < 8; ++i)
                    {
                        if (leaf.valid[i]) mask |= (1 << i);
                    }

                    auto it = leaf_dict.find(mask);
                    if (it != leaf_dict.end())
                    {
                        return it->second;
                    }

                    u32 idx = geometry_leaves_.size();
                    leaf_dict[mask] = idx;
                    geometry_leaves_.push_back(mask);
                    return idx;
                };

            std::vector<std::vector<u32>> level_buffers(depth);
            std::vector<TempNode> temp_nodes(depth);
            TempNode temp_leaf;

#pragma pack(push, 1)
            struct VoxelData
            {
                u64 morton;
                Voxel voxel;
            };
#pragma pack(pop)

            VoxelData voxel_data;
            size_t total_voxels = std::filesystem::file_size(filepath) / sizeof(VoxelData);
            size_t processed_voxels = 0;

            auto calculate_divergence = [](u64 morton_a, u64 morton_b) -> int
                {
                    u64 diff = morton_a ^ morton_b;
                    if (diff == 0) return 0;
                    return (63 - std::countl_zero(diff)) / 3;
                };

            if (!file.read(reinterpret_cast<char*>(&voxel_data), sizeof(VoxelData)))
                return 0;

            processed_voxels++;

            u64 last_morton = voxel_data.morton;
            temp_leaf.add(last_morton & 0b111, 0);

            // --- FAZA 1 ---
            while (file.read(reinterpret_cast<char*>(&voxel_data), sizeof(VoxelData)))
            {
                processed_voxels++;
                if (processed_voxels % 10000 == 0)
                {
                    double percent = (static_cast<double>(processed_voxels) * 100.0) / total_voxels;
                    std::printf("\r[Phase 1] Building SVO: %zu/%zu [%.2f%%]", processed_voxels, total_voxels, percent);
                }

                if ((last_morton >> 3) == (voxel_data.morton >> 3))
                {
                    temp_leaf.add(voxel_data.morton & 0b111, 0);
                }
                else
                {
                    u32 node_idx = get_or_add_leaf(temp_leaf);
                    temp_leaf.clear();

                    int current_level = 0;
                    temp_nodes[current_level].add((last_morton >> 3) & 0b111, node_idx);

                    int max_level = calculate_divergence(last_morton, voxel_data.morton);
                    max_level = std::min(max_level, (int)depth - 1);

                    while (current_level < max_level - 1)
                    {
                        node_idx = temp_nodes[current_level].emit(level_buffers[current_level + 1]);
                        temp_nodes[current_level].clear();
                        ++current_level;

                        if (current_level >= depth - 1) break;

                        u8 parent_octant = (last_morton >> (3 * (current_level + 1))) & 0b111;
                        temp_nodes[current_level].add(parent_octant, node_idx);
                    }

                    temp_leaf.add(voxel_data.morton & 0b111, 0);
                }
                last_morton = voxel_data.morton;
            }
            std::cout << "\n";

            if (!temp_leaf.is_empty())
            {
                u32 node_idx = get_or_add_leaf(temp_leaf);
                temp_leaf.clear();
                temp_nodes[0].add((last_morton >> 3) & 0b111, node_idx);
            }

            for (int current_level = 0; current_level < depth - 1; ++current_level)
            {
                if (!temp_nodes[current_level].is_empty())
                {
                    u32 node_idx = temp_nodes[current_level].emit(level_buffers[current_level + 1]);
                    temp_nodes[current_level].clear();

                    if (current_level + 1 < depth - 1)
                    {
                        u8 parent_octant = (last_morton >> (3 * (current_level + 2))) & 0b111;
                        temp_nodes[current_level + 1].add(parent_octant, node_idx);
                    }
                }
            }

            // --- FAZA 2 ---
            std::unordered_map<u32, u32> remap_table;
            u32 last_root_index = 0;

            for (int d = 1; d < depth; ++d) // WAŻNE: Od d=1
            {
                std::printf("\r[Phase 2] Compressing level %d/%d", d + 1, depth);
                auto& current_level = level_buffers[d];
                if (current_level.empty()) continue;

                std::vector<u32> node_starts;
                u32 i = 0;
                while (i < current_level.size())
                {
                    node_starts.push_back(i);
                    u32 desc = current_level[i];
                    u32 old_child_count = (desc == 0) ? 0 : (max_offset(desc) + 1);

                    if (d > 1)
                    {
                        u32 new_desc = 0;
                        std::vector<u32> unique_children;

                        for (int oct = 0; oct < 8; ++oct)
                        {
                            u32 chunk = (desc >> (oct * 4)) & 0xF;
                            if (chunk & 0b1000)
                            {
                                u8 old_offset = chunk & 0b0111;
                                u32 old_child_ptr = current_level[i + 1 + old_offset];
                                u32 patched_ptr = remap_table.at(old_child_ptr);

                                u32 new_offset = 0;
                                bool found = false;
                                for (size_t j = 0; j < unique_children.size(); ++j)
                                {
                                    if (unique_children[j] == patched_ptr)
                                    {
                                        new_offset = j;
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    new_offset = unique_children.size();
                                    unique_children.push_back(patched_ptr);
                                }

                                u32 new_chunk = 0b1000 | (new_offset & 0b0111);
                                new_desc |= (new_chunk << (oct * 4));
                            }
                        }

                        current_level[i] = new_desc;
                        for (size_t j = 0; j < unique_children.size(); ++j)
                        {
                            current_level[i + 1 + j] = unique_children[j];
                        }
                    }

                    i += 1 + old_child_count;
                }

                std::sort(node_starts.begin(), node_starts.end(), [&](u32 a, u32 b)
                    {
                        u32 descA = current_level[a];
                        u32 descB = current_level[b];

                        if (descA != descB) return descA < descB;

                        u32 child_count = (descA == 0) ? 0 : (max_offset(descA) + 1);
                        for (u32 c = 1; c <= child_count; ++c)
                        {
                            if (current_level[a + c] != current_level[b + c])
                            {
                                return current_level[a + c] < current_level[b + c];
                            }
                        }
                        return false;
                    });

                std::unordered_map<u32, u32> new_remap;

                u32 first_old = node_starts[0];
                u32 first_desc = current_level[first_old];
                u32 first_child_count = (first_desc == 0) ? 0 : (max_offset(first_desc) + 1);

                u32 current_compact_start = nodes_.size();
                new_remap[first_old] = current_compact_start;
                last_root_index = current_compact_start;

                nodes_.push_back(first_desc);
                for (u32 c = 1; c <= first_child_count; ++c) nodes_.push_back(current_level[first_old + c]);

                u32 last_unique_compact_start = current_compact_start;

                for (size_t s = 1; s < node_starts.size(); ++s)
                {
                    u32 curr_old = node_starts[s];
                    u32 prev_old = node_starts[s - 1];

                    bool is_same = true;
                    u32 desc = current_level[curr_old];
                    if (desc != current_level[prev_old])
                    {
                        is_same = false;
                    }
                    else
                    {
                        u32 child_count = (desc == 0) ? 0 : (max_offset(desc) + 1);
                        for (u32 c = 1; c <= child_count; ++c)
                        {
                            if (current_level[curr_old + c] != current_level[prev_old + c])
                            {
                                is_same = false;
                                break;
                            }
                        }
                    }

                    if (is_same)
                    {
                        new_remap[curr_old] = last_unique_compact_start;
                    }
                    else
                    {
                        u32 new_compact_start = nodes_.size();
                        new_remap[curr_old] = new_compact_start;
                        last_unique_compact_start = new_compact_start;
                        last_root_index = new_compact_start;

                        u32 child_count = (desc == 0) ? 0 : (max_offset(desc) + 1);
                        nodes_.push_back(desc);
                        for (u32 c = 1; c <= child_count; ++c) nodes_.push_back(current_level[curr_old + c]);
                    }
                }

                remap_table = std::move(new_remap);
            }
            std::cout << "\nBuild complete!\n";

            return nodes_.empty() ? 0 : last_root_index;
        }

        // =========================================================================
        // INTERSECTOR (Raytracing)
        // =========================================================================

        Dag::Hit Dag::intersect(const Ray& ray, u8 depth, u32 root_index) const noexcept
        {
            constexpr float INF = std::numeric_limits<float>::infinity();

            glm::vec3 O = ray.origin;
            glm::vec3 invD = ray.direction_inverse;
            std::uint8_t a = 0;

            if (invD.x < 0.0f) { O.x = -O.x; invD.x = -invD.x; a |= 1; }
            if (invD.y < 0.0f) { O.y = -O.y; invD.y = -invD.y; a |= 2; }
            if (invD.z < 0.0f) { O.z = -O.z; invD.z = -invD.z; a |= 4; }

            float t_enter = 0.0f;
            float t_exit = INF;
            const float root_half = static_cast<float>(1u << depth);

            float t0x = (-root_half - O.x) * invD.x;
            float t1x = (root_half - O.x) * invD.x;
            t_enter = std::max(t_enter, t0x);
            t_exit = std::min(t_exit, t1x);

            float t0y = (-root_half - O.y) * invD.y;
            float t1y = (root_half - O.y) * invD.y;
            t_enter = std::max(t_enter, t0y);
            t_exit = std::min(t_exit, t1y);

            float t0z = (-root_half - O.z) * invD.z;
            float t1z = (root_half - O.z) * invD.z;
            t_enter = std::max(t_enter, t0z);
            t_exit = std::min(t_exit, t1z);

            t_enter = std::max(t_enter, 0.0f);

            if (t_exit < t_enter)
                return { .t = INF, .normal = glm::vec3{0}, .voxel = {.rgbe = 0} };

            std::uint8_t last_axis = 0;
            {
                float best = t0x;
                last_axis = 0;
                if (t0y > best) { best = t0y; last_axis = 1; }
                if (t0z > best) { last_axis = 2; }
            }

            StackFrame stack[32];
            int sp = 0;

            u32 current_node_idx = root_index;
            float half = root_half;

            float txm = (-O.x) * invD.x;
            float tym = (-O.y) * invD.y;
            float tzm = (-O.z) * invD.z;

            std::uint8_t oct_idx = (std::uint8_t(txm <= t_enter) << 0) |
                (std::uint8_t(tym <= t_enter) << 1) |
                (std::uint8_t(tzm <= t_enter) << 2);

            float tx = (txm > t_enter) ? txm : INF;
            float ty = (tym > t_enter) ? tym : INF;
            float tz = (tzm > t_enter) ? tzm : INF;

            while (true)
            {
                bool hit_voxel = false;
                std::uint8_t true_oct = oct_idx ^ a;
                u32 child_ptr = 0;
                bool descend = false;

                // 1. ODCZYT W ZALEŻNOŚCI OD GŁĘBOKOŚCI
                // Musimy wiedzieć z jakiej tablicy czytać ZANIM pobierzemy dane!
                if (depth > 1)
                {
                    // Węzły wewnętrzne (poziom 2 i wyżej)
                    u32 desc = nodes_[current_node_idx];
                    u32 child_chunk = (desc >> (4 * true_oct)) & 0xF;
                    if (child_chunk & 0b1000)
                    {
                        u32 offset = child_chunk & 0b0111;
                        child_ptr = nodes_[current_node_idx + 1 + offset];
                        descend = true; // Znalazł drogę w dół!
                    }
                }
                else
                {
                    // depth == 1! current_node_idx to TERAZ indeks do tablicy liści!
                    u8 leaf_mask = geometry_leaves_[current_node_idx];
                    if (leaf_mask & (1 << true_oct))
                    {
                        hit_voxel = true; // BUM! Uderzenie w geometrię woksela.
                    }
                }

                // 2. OBSŁUGA TRAFIENIA
                if (hit_voxel)
                {
                    Voxel v;
                    v.rgbe = 0xFFFFFFFF; // Biały wypełniacz

                    glm::vec3 n{ 0.f, 0.f, 0.f };
                    if (last_axis == 0) n.x = -1.f;
                    else if (last_axis == 1) n.y = -1.f;
                    else n.z = -1.f;

                    if (a & 1) n.x = -n.x;
                    if (a & 2) n.y = -n.y;
                    if (a & 4) n.z = -n.z;

                    return { .t = t_enter, .normal = n, .voxel = v };
                }

                // 3. SCHODZENIE W DÓŁ (DESCEND)
                if (descend)
                {
                    float t_next = tx;
                    if (ty < t_next) t_next = ty;
                    if (tz < t_next) t_next = tz;

                    const float child_t_exit = (t_exit < t_next) ? t_exit : t_next;

                    stack[sp++] = { current_node_idx, tx, ty, tz, t_exit, oct_idx };

                    half *= 0.5f;
                    const float offset_pos = half;

                    txm += ((oct_idx & 1) ? +offset_pos : -offset_pos) * invD.x;
                    tym += ((oct_idx & 2) ? +offset_pos : -offset_pos) * invD.y;
                    tzm += ((oct_idx & 4) ? +offset_pos : -offset_pos) * invD.z;

                    current_node_idx = child_ptr; // Wskaźnik z nodes_ -> na kolejny nodes_ LUB geometry_leaves_
                    t_exit = child_t_exit;
                    --depth;

                    oct_idx = (std::uint8_t(txm <= t_enter) << 0) |
                        (std::uint8_t(tym <= t_enter) << 1) |
                        (std::uint8_t(tzm <= t_enter) << 2);

                    tx = (txm > t_enter) ? txm : INF;
                    ty = (tym > t_enter) ? tym : INF;
                    tz = (tzm > t_enter) ? tzm : INF;

                    continue;
                }

                // 4. PRZESUNIĘCIE / WYJŚCIE W GÓRĘ (ADVANCE / POP)
                while (true)
                {
                    int axis = 0;
                    float t_next = tx;
                    if (ty < t_next) { t_next = ty; axis = 1; }
                    if (tz < t_next) { t_next = tz; axis = 2; }

                    if (t_next > t_exit)
                    {
                        if (sp == 0)
                            return { .t = INF, .normal = glm::vec3{0}, .voxel = {.rgbe = 0} };

                        ++depth;
                        const float offset_pos = half;

                        const StackFrame& frame = stack[--sp];
                        current_node_idx = frame.node_idx;
                        tx = frame.tx;
                        ty = frame.ty;
                        tz = frame.tz;
                        t_exit = frame.t_exit;
                        oct_idx = frame.oct_idx;

                        txm -= ((oct_idx & 1) ? +offset_pos : -offset_pos) * invD.x;
                        tym -= ((oct_idx & 2) ? +offset_pos : -offset_pos) * invD.y;
                        tzm -= ((oct_idx & 4) ? +offset_pos : -offset_pos) * invD.z;

                        half *= 2.0f;
                        continue;
                    }

                    oct_idx |= std::uint8_t(1u << axis);
                    if (axis == 0) tx = INF;
                    else if (axis == 1) ty = INF;
                    else tz = INF;

                    t_enter = t_next;
                    last_axis = static_cast<std::uint8_t>(axis);
                    break;
                }
            }
        }

    } // namespace v2
} // namespace vrt