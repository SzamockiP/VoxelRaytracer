#include <vrt/accel/dag_v2.hpp>
#include <iostream>
#include <unordered_map>
#include <fstream>
#include <algorithm>

namespace vrt
{
    namespace v2
    {
#pragma pack(push, 1)
        struct BaertNode
        {
            uint64_t data_address;       // 8 bajtów (Ignorujemy, to kolory/normalne)
            uint64_t children_base;      // 8 bajtów (Baza dzieci)
            int8_t child_offsets[8];     // 8 bajtów (Offsety od bazy, -1 oznacza brak)
        };
#pragma pack(pop)

        // =========================================================================
        // STRUKTURY POMOCNICZE DO KOMPRESJI V3
        // =========================================================================

        struct InternalNodeData
        {
            vrt::u32 desc;
            std::vector<vrt::u32> children;

            bool operator==(const InternalNodeData& other) const
            {
                return desc == other.desc && children == other.children;
            }
        };

        struct InternalNodeHash
        {
            size_t operator()(const InternalNodeData& data) const
            {
                size_t hash = std::hash<vrt::u32>()(data.desc);
                for (vrt::u32 c : data.children)
                {
                    hash ^= std::hash<vrt::u32>()(c) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
                }
                return hash;
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
        // BUILDER: IMPORT Z .OCTREE I NATYCHMIASTOWA KOMPRESJA V3
        // =========================================================================

        vrt::u32 Dag::build(u8 fallback_depth, const std::filesystem::path& filepath)
        {
            if (!std::filesystem::exists(filepath))
                throw std::runtime_error("Error: file doesn't exist: " + filepath.string());

            // 1. CZYTANIE NAGŁÓWKA (.octree)
            std::ifstream header_file(filepath);
            std::string token;
            vrt::u32 gridlength = 0;
            size_t n_nodes = 0;

            while (header_file >> token)
            {
                if (token == "gridlength") header_file >> gridlength;
                if (token == "n_nodes") header_file >> n_nodes;
            }
            header_file.close();

            if (gridlength == 0 || n_nodes == 0)
                throw std::runtime_error("Error: Nieudane parsowanie naglowka .octree");

            std::printf("[V3 Builder] Wczytywanie modelu %dx%dx%d\n", gridlength, gridlength, gridlength);
            std::printf("[V3 Builder] Calkowita liczba wezlow SVO: %zu\n", n_nodes);

            // 2. WCZYTYWANIE CAŁEGO SVO DO PAMIĘCI
            std::filesystem::path nodes_path = filepath;
            nodes_path.replace_extension(".octreenodes");

            if (!std::filesystem::exists(nodes_path))
                throw std::runtime_error("Error: Brakuje pliku " + nodes_path.string());

            std::vector<BaertNode> svo_nodes(n_nodes);
            std::ifstream nodes_file(nodes_path, std::ios::binary);
            nodes_file.read(reinterpret_cast<char*>(svo_nodes.data()), n_nodes * sizeof(BaertNode));
            nodes_file.close();

            // 3. MATEMATYCZNE SZUKANIE KORZENIA (Root Finder)
            std::printf("[V3 Builder] Skanowanie grafu w poszukiwaniu Korzenia...\n");

            u8 expected_depth = fallback_depth;
            if (gridlength > 0) {
                expected_depth = 0;
                u32 tmp = gridlength;
                while (tmp > 1) { tmp >>= 1; expected_depth++; }
            }

            u32 root_idx = 0;
            bool root_found = false;
            u8 highest_depth_found = 0;

            for (int i = static_cast<int>(n_nodes) - 1; i >= 0; --i)
            {
                u8 current_max = 0;
                std::vector<std::pair<u32, u8>> q;
                q.reserve(100);
                q.push_back({static_cast<u32>(i), 0});
                
                while (!q.empty()) {
                    auto [node, d] = q.back();
                    q.pop_back();
                    
                    if (d > current_max) current_max = d;
                    if (current_max >= expected_depth) break; // Znalazł pełne drzewo
                    
                    for (int k = 0; k < 8; ++k) {
                        if (svo_nodes[node].child_offsets[k] >= 0) {
                            u32 cidx = static_cast<u32>(svo_nodes[node].children_base + svo_nodes[node].child_offsets[k]);
                            if (cidx < n_nodes) {
                                q.push_back({cidx, d + 1});
                            }
                        }
                    }
                }
                
                if (current_max > highest_depth_found) {
                    highest_depth_found = current_max;
                    root_idx = i;
                    root_found = true;
                    
                    if (highest_depth_found >= expected_depth) {
                        break; 
                    }
                }
            }

            if (!root_found)
            {
                std::printf("[BŁĄD] Plik jest uszkodzony! Nie znaleziono zadnego Korzenia.\n");
                return 0;
            }
            std::printf("[V3 Builder] Korzen znaleziony pod indeksem: %u (zweryfikowana glebokosc: %d)\n", root_idx, highest_depth_found);

            // 4. SONDA GŁĘBINOWA Z ITERACJĄ (Usunięta: polegamy na głębokości z wywołania lub wielkości siatki)
            u8 max_depth = expected_depth; // Zabezpieczenie zgodności z intersect()

            std::printf("[V3 Builder] Rzeczywista fizyczna glebokosc SVO to: %d\n", max_depth);

            // Definiujemy poziom, na którym budujemy 64-bitowe liście (2 poziomy nad dnem)
            u8 target_leaf_depth = (max_depth >= 2) ? (max_depth - 2) : 0;

            // 5. PRZYGOTOWANIE STRUKTUR V3
            nodes_.clear();
            geometry_leaves_.clear();

            std::unordered_map<u64, u32> leaf_dict;
            std::unordered_map<InternalNodeData, u32, InternalNodeHash> node_dict;

            auto build_solid_dag = [&](auto& self_solid, u8 current_depth) -> u32 {
                if (current_depth == target_leaf_depth) {
                    u64 mask = 0xFFFFFFFFFFFFFFFFULL;
                    auto it = leaf_dict.find(mask);
                    if (it != leaf_dict.end()) return it->second;
                    u32 new_idx = geometry_leaves_.size();
                    geometry_leaves_.push_back(mask);
                    leaf_dict[mask] = new_idx;
                    return new_idx;
                }
                InternalNodeData node_data;
                node_data.desc = 0;
                u32 child_dag_idx = self_solid(self_solid, current_depth + 1);
                node_data.children.push_back(child_dag_idx);
                for (int i = 0; i < 8; ++i) {
                    u32 chunk = 0b1000 | 0b0000;
                    node_data.desc |= (chunk << (i * 4));
                }
                auto it = node_dict.find(node_data);
                if (it != node_dict.end()) return it->second;
                u32 new_idx = nodes_.size();
                nodes_.push_back(node_data.desc);
                for (u32 c : node_data.children) nodes_.push_back(c);
                node_dict[node_data] = new_idx;
                return new_idx;
            };

            std::printf("[V3 Builder] Rozpoczynam miazdzenie algorytmem DAG V3 (Post-Order DFS)...\n");

            // 6. REKURENCYJNE PRZESZUKIWANIE (DFS)
            auto dfs = [&](auto& self, u32 node_idx, u8 current_depth) -> u32
                {
                    const BaertNode& n = svo_nodes[node_idx];
                    bool has_any_child = false;
                    for (int i = 0; i < 8; ++i) {
                        if (n.child_offsets[i] >= 0) {
                            u32 cidx = static_cast<u32>(n.children_base + n.child_offsets[i]);
                            if (cidx < n_nodes) {
                                has_any_child = true; break;
                            }
                        }
                    }

                    if (!has_any_child && n.data_address != 0) { // 0 in this struct format means NULL data ptr
                        return build_solid_dag(build_solid_dag, current_depth);
                    }

                    if (current_depth == target_leaf_depth)
                    {
                        u64 mask = 0;
                        const BaertNode& n1 = svo_nodes[node_idx];

                        for (int i = 0; i < 8; ++i)
                        {
                            if (n1.child_offsets[i] >= 0)
                            {
                                u32 child1_idx = static_cast<u32>(n1.children_base + n1.child_offsets[i]);
                                if (child1_idx < n_nodes) {
                                    const BaertNode& n2 = svo_nodes[child1_idx];

                                    bool has_any_child2 = false;
                                    for(int j = 0; j < 8; ++j) {
                                        if(n2.child_offsets[j] >= 0) {
                                            u32 cidx2 = static_cast<u32>(n2.children_base + n2.child_offsets[j]);
                                            if(cidx2 < n_nodes) {
                                                has_any_child2 = true; break;
                                            }
                                        }
                                    }

                                    if (!has_any_child2 && n2.data_address != 0) {
                                        for (int j = 0; j < 8; ++j) {
                                            mask |= (1ULL << ((i * 8) + j));
                                        }
                                    } else {
                                        for (int j = 0; j < 8; ++j)
                                        {
                                            if (n2.child_offsets[j] >= 0)
                                            {
                                                u32 child2_idx = static_cast<u32>(n2.children_base + n2.child_offsets[j]);
                                                if (child2_idx < n_nodes) {
                                                    mask |= (1ULL << ((i * 8) + j));
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        auto it = leaf_dict.find(mask);
                        if (it != leaf_dict.end()) return it->second;

                        u32 new_idx = geometry_leaves_.size();
                        geometry_leaves_.push_back(mask);
                        leaf_dict[mask] = new_idx;
                        return new_idx;
                    }

                    // Węzły Wewnętrzne (Budowa 32-bitowego deskryptora)
                    InternalNodeData node_data;
                    node_data.desc = 0;
                    for (int i = 0; i < 8; ++i)
                    {
                        if (n.child_offsets[i] >= 0)
                        {
                            u32 child_idx = static_cast<u32>(n.children_base + n.child_offsets[i]);
                            if (child_idx < n_nodes) {
                                u32 child_dag_idx = self(self, child_idx, current_depth + 1);

                            u32 offset = 0;
                            auto it = std::find(node_data.children.begin(), node_data.children.end(), child_dag_idx);
                            if (it != node_data.children.end())
                            {
                                offset = std::distance(node_data.children.begin(), it);
                            }
                            else
                            {
                                offset = node_data.children.size();
                                node_data.children.push_back(child_dag_idx);
                            }

                            u32 chunk = 0b1000 | (offset & 0b0111);
                            node_data.desc |= (chunk << (i * 4));
                            }
                        }
                    }

                    auto it = node_dict.find(node_data);
                    if (it != node_dict.end()) return it->second;

                    u32 new_idx = nodes_.size();
                    nodes_.push_back(node_data.desc);
                    for (u32 c : node_data.children) nodes_.push_back(c);

                    node_dict[node_data] = new_idx;
                    return new_idx;
                };

            u32 final_root_idx = dfs(dfs, root_idx, 0);

            std::printf("[V3 Builder] Kompresja zakonczona! \n\n");
            return final_root_idx;
        }

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

                // ==========================================================
                // 1. ODCZYT W ZALEŻNOŚCI OD GŁĘBOKOŚCI (WIRTUALNE DRZEWO V3)
                // ==========================================================

                if (depth > 2) // <--- NAPRAWIONE! Węzły wewnętrzne kończą się wyżej!
                {
                    // Węzły Wewnętrzne (Odczyt z tablicy 32-bitowej nodes_)
                    u32 desc = nodes_[current_node_idx];
                    u32 child_chunk = (desc >> (4 * true_oct)) & 0xF;
                    if (child_chunk & 0b1000)
                    {
                        u32 offset = child_chunk & 0b0111;
                        child_ptr = nodes_[current_node_idx + 1 + offset];
                        descend = true;
                    }
                }
                else if (depth == 2) // <--- WIRTUALNY POZIOM 1 (Górna część u64)
                {
                    // Tutaj wchodzimy w nasz spakowany blok 4x4x4.
                    // Analizujemy u64 jako 8 bloków po 8 bitów. Szukamy odpowiedniego bajta.
                    u64 leaf_mask = geometry_leaves_[current_node_idx];
                    u8 chunk = (leaf_mask >> (true_oct * 8)) & 0xFF;

                    if (chunk != 0)
                    {
                        // Blok 2x2x2 nie jest pusty! "Schodzimy" głębiej. 
                        // Indeks zostaje TEN SAM, bo fizycznie nie zmieniamy miejsca w pamięci RAM!
                        child_ptr = current_node_idx;
                        descend = true;
                    }
                }
                else // depth == 1 <--- WIRTUALNY POZIOM 0 (Dolna część u64)
                {
                    // Sprawdzamy konkretny bit wewnątrz wybranego wcześniej 1 bajta
                    u64 leaf_mask = geometry_leaves_[current_node_idx];

                    // Magia: Odzyskujemy oktant rodzica ze stosu, by wiedzieć który z 8 bajtów sprawdzić
                    std::uint8_t parent_true_oct = stack[sp - 1].oct_idx ^ a;
                    u8 chunk = (leaf_mask >> (parent_true_oct * 8)) & 0xFF;

                    if (chunk & (1 << true_oct))
                    {
                        hit_voxel = true; // Bezpośrednie trafienie w sam bit geometrii!
                    }
                }

                // ==========================================================
                // 2. OBSŁUGA TRAFIENIA
                // ==========================================================
                if (hit_voxel)
                {
                    Voxel v;
                    v.rgbe = 0xFFFFFFFF; // Biały piksel geometrii

                    glm::vec3 n{ 0.f, 0.f, 0.f };
                    if (last_axis == 0) n.x = -1.f;
                    else if (last_axis == 1) n.y = -1.f;
                    else n.z = -1.f;

                    if (a & 1) n.x = -n.x;
                    if (a & 2) n.y = -n.y;
                    if (a & 4) n.z = -n.z;

                    return { .t = t_enter, .normal = n, .voxel = v };
                }

                // ==========================================================
                // 3. SCHODZENIE W DÓŁ (DESCEND)
                // ==========================================================
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

                    current_node_idx = child_ptr;
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

                // ==========================================================
                // 4. PRZESUNIĘCIE / WYJŚCIE W GÓRĘ (ADVANCE / POP)
                // ==========================================================
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