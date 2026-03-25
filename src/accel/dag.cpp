#include <algorithm>
#include <bit>
#include <cstdint>
#include <fstream>
#include <list>
#include <print>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vrt/accel/dag.hpp>


struct StackFrame
{
    vrt::Dag::Node node;
    float tx, ty, tz;
    float t_exit;
    vrt::u8 oct_idx;
};

vrt::Dag::Hit vrt::Dag::intersect(const Ray& ray, u8 depth,
    const Node& root) const noexcept
{
    constexpr float INF = std::numeric_limits<float>::infinity();

    // mirror ray
    glm::vec3 O = ray.origin;
    glm::vec3 invD = ray.direction_inverse;
    std::uint8_t a = 0; // axis inversion mask

    if (invD.x < 0.0f)
    {
        O.x = -O.x;
        invD.x = -invD.x;
        a |= 1;
    }
    if (invD.y < 0.0f)
    {
        O.y = -O.y;
        invD.y = -invD.y;
        a |= 2;
    }
    if (invD.z < 0.0f)
    {
        O.z = -O.z;
        invD.z = -invD.z;
        a |= 4;
    }

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
        if (t0y > best)
        {
            best = t0y;
            last_axis = 1;
        }
        if (t0z > best)
        {
            last_axis = 2;
        }
    }

    StackFrame stack[32];
    int sp = 0;

    Node current_node = root;

    float half = root_half;

    // Czas przejścia promienia przez środek korzenia wzdłuż każdej osi
    float txm = (-O.x) * invD.x;
    float tym = (-O.y) * invD.y;
    float tzm = (-O.z) * invD.z;

    // Startowy oktant: bit=1 jeśli środek węzła jest już za nami na danej osi
    std::uint8_t oct_idx = (std::uint8_t(txm <= t_enter) << 0) |
        (std::uint8_t(tym <= t_enter) << 1) |
        (std::uint8_t(tzm <= t_enter) << 2);

    // Czas wyjścia z bieżącego oktantu wzdłuż każdej osi (INF jeśli już przekroczony)
    float tx = (txm > t_enter) ? txm : INF;
    float ty = (tym > t_enter) ? tym : INF;
    float tz = (tzm > t_enter) ? tzm : INF;

    while (true)
    {
        // 0. EARLY RAY TERMINATION (Lity, ucięty sześcian na wyższym poziomie)
        if (current_node.is_leaf())
        {
            glm::vec3 n{ 0.f, 0.f, 0.f };
            if (last_axis == 0)
                n.x = -1.f;
            else if (last_axis == 1)
                n.y = -1.f;
            else
                n.z = -1.f;

            if (a & 1)
                n.x = -n.x;
            if (a & 2)
                n.y = -n.y;
            if (a & 4)
                n.z = -n.z;

            return { .t = t_enter, .normal = n, .voxel = current_node.voxel };
        }

        // 1. ODCZYT Z 4-BITOWEGO DESKRYPTORA
        std::uint8_t true_oct = oct_idx ^ a;
        u32 child_data = (current_node.descriptor >> (4 * true_oct)) & 0xF;

        // Jeśli w danym oktancie cokolwiek jest (Bit validacji zapalony)
        if (child_data & 0b1000)
        {
            u32 offset = child_data & 0b0111;
            std::uint32_t child_index = current_node.index + offset;

            // 2. SCHODZENIE W DÓŁ
            if (depth > 1)
            {
                float t_next = tx;
                if (ty < t_next)
                    t_next = ty;
                if (tz < t_next)
                    t_next = tz;

                const float child_t_exit = (t_exit < t_next) ? t_exit : t_next;

                // Odkładamy ojca na stos
                stack[sp++] = { current_node, tx, ty, tz, t_exit, oct_idx };

                half *= 0.5f;
                const float offset_pos = half;

                txm += ((oct_idx & 1) ? +offset_pos : -offset_pos) * invD.x;
                tym += ((oct_idx & 2) ? +offset_pos : -offset_pos) * invD.y;
                tzm += ((oct_idx & 4) ? +offset_pos : -offset_pos) * invD.z;

                current_node = nodes_[child_index];
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
            else
            {
                // Jesteśmy na depth == 0, czyli uderzyliśmy w wektor `leaves_`
                Voxel v = leaves_[child_index];

                glm::vec3 n{ 0.f, 0.f, 0.f };
                if (last_axis == 0)
                    n.x = -1.f;
                else if (last_axis == 1)
                    n.y = -1.f;
                else
                    n.z = -1.f;

                if (a & 1)
                    n.x = -n.x;
                if (a & 2)
                    n.y = -n.y;
                if (a & 4)
                    n.z = -n.z;

                return { .t = t_enter, .normal = n, .voxel = v };
            }
        }

        // 3. PRZESUNIĘCIE / WYJŚCIE W GÓRĘ (ADVANCE / POP)
        while (true)
        {
            int axis = 0;
            float t_next = tx;
            if (ty < t_next)
            {
                t_next = ty;
                axis = 1;
            }
            if (tz < t_next)
            {
                t_next = tz;
                axis = 2;
            }

            // POP (Wyjście z obecnego sześcianu)
            if (t_next > t_exit)
            {
                if (sp == 0)
                    return { .t = INF, .normal = glm::vec3{0}, .voxel = {.rgbe = 0} };

                ++depth;
                const float offset_pos = half;

                const StackFrame& frame = stack[--sp];
                current_node = frame.node;
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

            // ADVANCE (Krok do kolejnego oktantu wewnątrz ojca)
            oct_idx |= std::uint8_t(1u << axis);
            if (axis == 0)
                tx = INF;
            else if (axis == 1)
                ty = INF;
            else
                tz = INF;

            t_enter = t_next;
            last_axis = static_cast<std::uint8_t>(axis);
            break;
        }
    }
}

template <typename T> inline size_t hash_memory(const T& val)
{
    return std::hash<std::string_view>{}(
        std::string_view(reinterpret_cast<const char*>(&val), sizeof(T)));
}

template <typename T> struct TempNode
{
    vrt::u8 mask;
    std::vector<T> elements;

    TempNode()
    {
        mask = 0;
        elements.reserve(8);
    }

    void add(vrt::u8 octant, const T& element)
    {
        mask |= (1 << octant);
        elements.push_back(element);
    }

    void clear()
    {
        mask = 0;
        elements.clear();
    }

    bool is_empty() const { return mask == 0; }
};

inline void hash_combine(size_t& seed, size_t hash_val)
{
    seed ^= hash_val + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T>
vrt::Dag::Node insert_with_sliding_window(
    const TempNode<T>& temp,
    std::vector<T>& target_buffer,
    std::unordered_map<size_t, std::vector<int>>& index_cache,
    std::unordered_map<size_t, vrt::Dag::Node>& memo_cache)
{
    int N = temp.elements.size();
    if (N == 0) return { 0 };
    if (N > 8) throw std::runtime_error("FATAL ERROR: TempNode N > 8");

    // 1. Memo Cache (Pamięć gotowych węzłów)
    size_t temp_hash = std::hash<uint8_t>{}(temp.mask);
    for (const auto& el : temp.elements) hash_combine(temp_hash, hash_memory(el));

    if (auto it = memo_cache.find(temp_hash); it != memo_cache.end())
        return it->second;

    // 2. Unikalne klocki (Krótko, czystym C++)
    std::vector<T> unique_elems;
    int elem_to_unique[8];
    for (int i = 0; i < N; ++i)
    {
        auto it = std::find(unique_elems.begin(), unique_elems.end(), temp.elements[i]);
        elem_to_unique[i] = std::distance(unique_elems.begin(), it);
        if (it == unique_elems.end()) unique_elems.push_back(temp.elements[i]);
    }
    int U = unique_elems.size();

    // 3. Przygotowanie do Galloping Search
    std::vector<const std::vector<int>*> vecs(U);
    bool can_gallop = true;
    for (int i = 0; i < U; ++i)
    {
        auto it = index_cache.find(hash_memory(unique_elems[i]));
        if (it == index_cache.end() || it->second.empty())
        {
            can_gallop = false; break; // Brakuje elementu w historii
        }
        vecs[i] = &it->second;
    }

    int best_index = -1;
    int unique_offsets[8] = { 0 };

    // 4. GALLOPING SEARCH (Skoki binarne)
    if (can_gallop)
    {
        std::vector<int> ptrs(U, 0);
        while (true)
        {
            int min_val = INT_MAX, max_val = -1, min_u = -1;
            for (int i = 0; i < U; ++i)
            {
                int val = (*vecs[i])[ptrs[i]];
                if (val < min_val) { min_val = val; min_u = i; }
                if (val > max_val) max_val = val;
            }

            if (max_val - min_val < 8)
            {
                best_index = min_val; // BINGO! Mamy idealne okienko.
                for (int i = 0; i < U; ++i) unique_offsets[i] = (*vecs[i])[ptrs[i]] - best_index;
                break;
            }

            auto& vec = *vecs[min_u];
            auto it = std::lower_bound(vec.begin() + ptrs[min_u], vec.end(), max_val - 7);
            if (it == vec.end()) break; // Koniec historii dla tego klocka
            ptrs[min_u] = std::distance(vec.begin(), it);
        }
    }

    // 5. FALLBACK (Jeśli Galloping nic nie znalazł)
    if (best_index == -1)
    {
        int overlap = 0;
        int max_o = std::min(U, (int)target_buffer.size());
        for (int o = max_o; o > 0; --o)
        {
            bool match = true;
            for (int i = 0; i < o; ++i)
            {
                if (!(target_buffer[target_buffer.size() - o + i] == unique_elems[i]))
                {
                    match = false; break;
                }
            }
            if (match) { overlap = o; break; }
        }

        best_index = target_buffer.size() - overlap;

        // Magiczna optymalizacja: w fallbacku offset unikalnego elementu to zawsze po prostu 'i'!
        for (int i = 0; i < U; ++i)
        {
            unique_offsets[i] = i;
            if (i >= overlap)
            {
                target_buffer.push_back(unique_elems[i]);
                index_cache[hash_memory(unique_elems[i])].push_back(target_buffer.size() - 1);
            }
        }
    }

    // 6. Budowanie i zapis węzła
    vrt::Dag::Node node;
    node.descriptor = 0;
    node.index = best_index;

    int active_idx = 0;
    for (int i = 0; i < 8; ++i)
    {
        if (temp.mask & (1 << i))
        {
            node.set_child_offset(i, unique_offsets[elem_to_unique[active_idx++]]);
        }
    }

    memo_cache[temp_hash] = node;
    return node;
}

vrt::Dag::Node vrt::Dag::build(u8 depth,
    const std::filesystem::path& filepath)
{
    if (!std::filesystem::exists(filepath))
    {
        throw std::runtime_error("Error: vrt::Dag::build filepath doesn't exist.");
    }

    std::ifstream file(filepath, std::ios::binary);
    // Mapy indeksów (Historia wystąpień na każdym poziomie)
    std::unordered_map<size_t, std::vector<int>> leaf_index_cache;
    std::vector<std::unordered_map<size_t, std::vector<int>>> node_index_caches(depth);

    // Mapy węzłów (Memoizacja ułożenia węzła na każdym poziomie)
    std::unordered_map<size_t, Node> leaf_memo_cache;
    std::vector<std::unordered_map<size_t, Node>> node_memo_caches(depth);

    std::vector<TempNode<Node>> temp_nodes(depth);
    TempNode<Voxel> temp_leaf;

    leaves_.clear();
    nodes_.clear();

    std::vector<std::vector<Node>> level_nodes(depth);

#pragma pack(push, 1)
    struct VoxelData
    {
        u64 morton;
        Voxel voxel;
    };
#pragma pack(pop)

    VoxelData voxel_data;
    size_t total_voxels =
        std::filesystem::file_size(filepath) / sizeof(VoxelData);
    size_t processed_voxels = 0;

    auto calculate_divergence = [](u64 morton_a, u64 morton_b) -> int
        {
            u64 diff = morton_a ^ morton_b;
            if (diff == 0)
                return 0;
            return (63 - std::countl_zero(diff)) / 3;
        };

    auto insert_leaf = [&](TempNode<Voxel>& leaves) -> Node
        {
            return insert_with_sliding_window(leaves, leaves_, leaf_index_cache,
                leaf_memo_cache);
        };

    auto insert_node = [&](TempNode<Node>& nodes, int level) -> Node
        {
            return insert_with_sliding_window(nodes, level_nodes[level],
                node_index_caches[level],
                node_memo_caches[level]);
        };

    if (!file.read(reinterpret_cast<char*>(&voxel_data), sizeof(VoxelData)))
        return Node{};
    processed_voxels++;

    u64 last_morton = voxel_data.morton;
    temp_leaf.add(last_morton & 0b111, voxel_data.voxel);

    while (file.read(reinterpret_cast<char*>(&voxel_data), sizeof(VoxelData)))
    {
        processed_voxels++;
        if (processed_voxels % 10000 == 0)
        {
            double percent =
                (static_cast<double>(processed_voxels) * 100.0) / total_voxels;
            std::print("\rProcessed voxels: {}/{} [{:.2f}%]", processed_voxels,
                total_voxels, percent);
        }

        if ((last_morton >> 3) == (voxel_data.morton >> 3))
        {
            temp_leaf.add(voxel_data.morton & 0b111, voxel_data.voxel);
        }
        else
        {
            Node node = insert_leaf(temp_leaf);
            temp_leaf.clear();

            int current_level = 0;
            temp_nodes[current_level].add((last_morton >> 3) & 0b111, node);

            int max_level = calculate_divergence(last_morton, voxel_data.morton);
            max_level = std::min(max_level, (int)depth - 1);

            while (current_level < max_level - 1)
            {
                node = insert_node(temp_nodes[current_level], current_level);
                temp_nodes[current_level].clear();
                ++current_level;

                if (current_level >= depth)
                    break;

                u8 parent_octant = (last_morton >> (3 * (current_level + 1))) & 0b111;
                temp_nodes[current_level].add(parent_octant, node);
            }

            temp_leaf.add(voxel_data.morton & 0b111, voxel_data.voxel);
        }

        last_morton = voxel_data.morton;
    }

    if (!temp_leaf.is_empty())
    {
        Node node = insert_leaf(temp_leaf);
        temp_leaf.clear();
        temp_nodes[0].add((last_morton >> 3) & 0b111, node);
    }

    // Domknięcie drzewa: przepychamy niezamknięte węzły od dołu do góry.
    // Po przetworzeniu wszystkich wokseli mogą zostać niepuste temp_nodes
    // na każdym poziomie — każdy musi zostać wstawiony i przekazany wyżej.
    for (int current_level = 0; current_level < depth; ++current_level)
    {
        if (!temp_nodes[current_level].is_empty())
        {
            Node node = insert_node(temp_nodes[current_level], current_level);
            temp_nodes[current_level].clear();

            if (current_level + 1 < depth)
            {
                int parent_octant = (last_morton >> (3 * (current_level + 2))) & 0b111;
                temp_nodes[current_level + 1].add(parent_octant, node);
            }
        }
    }

    size_t total_topology_nodes = 0;
    std::vector<size_t> level_offsets(depth, 0);

    for (int d = 0; d < depth; ++d)
    {
        level_offsets[d] = total_topology_nodes;
        total_topology_nodes += level_nodes[d].size();
    }

    nodes_.reserve(total_topology_nodes);

    // Scalamy wszystkie poziomy do jednej płaskiej tablicy nodes_.
    // Węzły były budowane per-poziom z lokalnymi indeksami — tu dodajemy
    // globalny offset, by każdy węzeł wskazywał poprawny index w nodes_.
    for (int d = 0; d < depth; ++d)
    {
        for (const Node& node : level_nodes[d])
        {
            Node patched_node = node;

            if (d > 0)
            {
                patched_node.index += level_offsets[d - 1];
            }

            nodes_.push_back(patched_node);
        }
    }

    return nodes_.empty() ? Node{} : nodes_.back();
}

void vrt::Dag::save(const std::filesystem::path& filepath) const
{
    if (nodes_.empty() && leaves_.empty())
        return;

    std::ofstream file(filepath, std::ios::binary);
    if (!file)
        throw std::runtime_error("Error: Cannot open file for writing: " + filepath.string());

    // Magic number: "VDAG"
    const char magic[4] = { 'V', 'D', 'A', 'G' };
    file.write(magic, 4);

    u64 num_nodes = nodes_.size();
    u64 num_leaves = leaves_.size();

    file.write(reinterpret_cast<const char*>(&num_nodes), sizeof(u64));
    file.write(reinterpret_cast<const char*>(&num_leaves), sizeof(u64));

    if (num_nodes > 0)
        file.write(reinterpret_cast<const char*>(nodes_.data()), num_nodes * sizeof(Node));
    if (num_leaves > 0)
        file.write(reinterpret_cast<const char*>(leaves_.data()), num_leaves * sizeof(Voxel));
}

vrt::Dag::Node vrt::Dag::load(const std::filesystem::path& filepath)
{
    std::ifstream file(filepath, std::ios::binary);
    if (!file)
        throw std::runtime_error("Error: Cannot open file for reading: " + filepath.string());

    char magic[4];
    file.read(magic, 4);
    if (magic[0] != 'V' || magic[1] != 'D' || magic[2] != 'A' || magic[3] != 'G')
        throw std::runtime_error("Error: Invalid VDAG file format.");

    u64 num_nodes = 0;
    u64 num_leaves = 0;

    file.read(reinterpret_cast<char*>(&num_nodes), sizeof(u64));
    file.read(reinterpret_cast<char*>(&num_leaves), sizeof(u64));

    nodes_.resize(num_nodes);
    leaves_.resize(num_leaves);

    if (num_nodes > 0)
        file.read(reinterpret_cast<char*>(nodes_.data()), num_nodes * sizeof(Node));
    if (num_leaves > 0)
        file.read(reinterpret_cast<char*>(leaves_.data()), num_leaves * sizeof(Voxel));

    return nodes_.empty() ? Node{} : nodes_.back();
}