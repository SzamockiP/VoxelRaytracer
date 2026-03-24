#include <vrt/accel/dag.hpp>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <bit>
#include <ranges>
#include <print>
#include <string_view>
#include <unordered_map>
#include <list>
#include <cstdint>

struct StackFrame
{
    vrt::Dag::Node node;
    float tx, ty, tz;
    float t_exit;
    vrt::u8 oct_idx;
};

vrt::Dag::Hit vrt::Dag::intersect(const Ray& ray, u8 depth, const Node& root) const noexcept
{
    constexpr float INF = std::numeric_limits<float>::infinity();

    // mirror ray
    glm::vec3 O = ray.origin;
    glm::vec3 invD = ray.direction_inverse;
    std::uint8_t a = 0; // axis inversion mask

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

    Node current_node = root;

    float half = root_half;

    float txm = (-O.x) * invD.x;
    float tym = (-O.y) * invD.y;
    float tzm = (-O.z) * invD.z;

    std::uint8_t oct_idx =
        (std::uint8_t(txm <= t_enter) << 0) |
        (std::uint8_t(tym <= t_enter) << 1) |
        (std::uint8_t(tzm <= t_enter) << 2);

    float tx = (txm > t_enter) ? txm : INF;
    float ty = (tym > t_enter) ? tym : INF;
    float tz = (tzm > t_enter) ? tzm : INF;

    while (true)
    {
        // 0. EARLY RAY TERMINATION (Lity, ucięty sześcian na wyższym poziomie)
        if (current_node.is_leaf())
        {
            glm::vec3 n{ 0.f, 0.f, 0.f };
            if (last_axis == 0) n.x = -1.f;
            else if (last_axis == 1) n.y = -1.f;
            else                     n.z = -1.f;

            if (a & 1) n.x = -n.x;
            if (a & 2) n.y = -n.y;
            if (a & 4) n.z = -n.z;

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
                if (ty < t_next) t_next = ty;
                if (tz < t_next) t_next = tz;

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

                oct_idx =
                    (std::uint8_t(txm <= t_enter) << 0) |
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
                if (last_axis == 0) n.x = -1.f;
                else if (last_axis == 1) n.y = -1.f;
                else                     n.z = -1.f;

                if (a & 1) n.x = -n.x;
                if (a & 2) n.y = -n.y;
                if (a & 4) n.z = -n.z;

                return { .t = t_enter, .normal = n, .voxel = v };
            }
        }

        // 3. PRZESUNIĘCIE / WYJŚCIE W GÓRĘ (ADVANCE / POP)
        while (true)
        {
            int axis = 0;
            float t_next = tx;
            if (ty < t_next) { t_next = ty; axis = 1; }
            if (tz < t_next) { t_next = tz; axis = 2; }

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
            if (axis == 0) tx = INF;
            else if (axis == 1) ty = INF;
            else                tz = INF;

            t_enter = t_next;
            last_axis = static_cast<std::uint8_t>(axis);
            break;
        }
    }
}

template <typename T>
inline size_t hash_memory(const T& val)
{
    return std::hash<std::string_view>{}(
        std::string_view(reinterpret_cast<const char*>(&val), sizeof(T))
        );
}

template <typename T>
struct TempNode
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

    bool is_empty() const
    {
        return mask == 0;
    }
};

struct PosHistory
{
    static constexpr int CAP = 32;
    int positions[CAP];
    std::uint8_t head = 0;
    std::uint8_t count = 0;

    void add(int pos)
    {
        positions[head] = pos;
        head = (head + 1) % CAP;
        if (count < CAP) count++;
    }
};

// LRU Cache oparty o generation counter. Każdy wpis ma timestamp ostatniego dostępu.
// Eviction wyrzuca wpis z najniższym timestampem.
// Unikamy iteratorów list jako typów pól — brak problemów z nested templates w clangd.
template <typename Key, typename Value>
class LruCache
{
private:
    struct Entry
    {
        Value         value;
        std::uint64_t last_used = 0;
    };

    std::unordered_map<Key, Entry> table;
    std::uint64_t clock = 0;
    size_t        cap;

public:
    explicit LruCache(size_t limit = 5000000) : cap(limit)
    {
        table.reserve(cap);
    }

    Value* get(const Key& key)
    {
        auto it = table.find(key);
        if (it == table.end()) return nullptr;
        it->second.last_used = ++clock;
        return &(it->second.value);
    }

    void put(const Key& key, const Value& val)
    {
        auto it = table.find(key);
        if (it != table.end())
        {
            it->second.value     = val;
            it->second.last_used = ++clock;
            return;
        }

        if (table.size() >= cap)
        {
            // Znajdź i wyrzuć LRU — wywołujemy rzadko, OVH jest akceptowalny
            auto lru = table.begin();
            for (auto jt = std::next(lru); jt != table.end(); ++jt)
                if (jt->second.last_used < lru->second.last_used) lru = jt;
            table.erase(lru);
        }

        table[key] = { val, ++clock };
    }
};

inline void hash_combine(size_t& seed, size_t hash_val)
{
    seed ^= hash_val + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T>
vrt::Dag::Node insert_with_sliding_window(
    const TempNode<T>& temp,
    std::vector<T>& target_buffer,
    LruCache<size_t, PosHistory>& index_cache,
    LruCache<size_t, vrt::Dag::Node>& memo_cache)
{
    int N = temp.elements.size();
    if (N == 0) return { 0 };
    if (N > 8) throw std::runtime_error("CRITICAL FATAL ERROR: TempNode N > 8!");

    size_t temp_hash = std::hash<uint8_t>{}(temp.mask);
    for (int e = 0; e < N; ++e)
    {
        hash_combine(temp_hash, hash_memory(temp.elements[e]));
    }

    vrt::Dag::Node* cached_node = memo_cache.get(temp_hash);
    if (cached_node != nullptr)
    {
        return *cached_node;
    }

    bool found_match = false;
    int best_index = -1;
    int best_offsets[8] = { 0 };

    PosHistory* history = index_cache.get(hash_memory(temp.elements[0]));
    if (history != nullptr)
    {
        for (int h = 0; h < history->count; ++h)
        {
            int pos = history->positions[h];
            int start_i = std::max(0, pos - 7);

            for (int i = start_i; i <= pos; ++i)
            {
                int window_size = std::min(8, (int)(target_buffer.size() - i));
                if (window_size < N) continue;

                bool window_has_all = true;
                int current_offsets[8] = { 0 };

                for (int e = 0; e < N; ++e)
                {
                    bool found_e = false;
                    for (int w = 0; w < window_size; ++w)
                    {
                        if (temp.elements[e] == target_buffer[i + w])
                        {
                            current_offsets[e] = w;
                            found_e = true;
                            break;
                        }
                    }
                    if (!found_e) { window_has_all = false; break; }
                }

                if (window_has_all)
                {
                    found_match = true;
                    best_index = i;
                    for (int e = 0; e < N; ++e) best_offsets[e] = current_offsets[e];
                    break;
                }
            }
            if (found_match) break;
        }
    }

    vrt::Dag::Node final_node;
    final_node.descriptor = 0;

    if (found_match)
    {
        final_node.index = best_index;
        int active_idx = 0;
        for (int i = 0; i < 8; ++i)
        {
            if (temp.mask & (1 << i))
            {
                final_node.set_child_offset(i, best_offsets[active_idx]);
                active_idx++;
            }
        }
    }
    else
    {
        // 1. Szukamy nakładania się początków na koniec (Prefix-Suffix Overlap)
        int overlap = 0;
        int max_possible_overlap = std::min(N, (int)target_buffer.size());

        for (int o = max_possible_overlap; o > 0; --o)
        {
            bool match = true;
            for (int i = 0; i < o; ++i)
            {
                // Używamy negacji operatora ==, żeby nie wymagać istnienia operatora != w strukturze Node/Voxel
                if (!(target_buffer[target_buffer.size() - o + i] == temp.elements[i]))
                {
                    match = false;
                    break;
                }
            }
            if (match)
            {
                overlap = o;
                break;
            }
        }

        // 2. Ustawiamy base_index na początek nakładającej się części w buforze
        final_node.index = target_buffer.size() - overlap;

        // 3. Dodajemy na koniec tylko te elementy, których nam brakuje
        for (int e = overlap; e < N; ++e)
        {
            target_buffer.push_back(temp.elements[e]);

            // Aktualizacja naszego EpochCache:
            PosHistory hist;
            PosHistory* existing = index_cache.get(hash_memory(temp.elements[e]));
            if (existing) hist = *existing;
            hist.add(target_buffer.size() - 1);
            index_cache.put(hash_memory(temp.elements[e]), hist);
        }

        // 4. Przypisujemy idealnie liniowe, gwarantowanie DODATNIE offsety
        int active_idx = 0;
        for (int i = 0; i < 8; ++i)
        {
            if (temp.mask & (1 << i))
            {
                final_node.set_child_offset(i, active_idx);
                active_idx++;
            }
        }
    }

    memo_cache.put(temp_hash, final_node);
    return final_node;
}

vrt::Dag::Node vrt::Dag::build(u8 depth, const std::filesystem::path& filepath)
{
    if (!std::filesystem::exists(filepath))
    {
        throw std::runtime_error("Error: vrt::Dag::build filepath doesn't exist.");
    }

    std::ifstream file(filepath, std::ios::binary);
    //size_t cache_limit = 5000000;
    //size_t cache_limit = 2500000;
    //size_t cache_limit = 1000000;
    size_t cache_limit = 500000;

    LruCache<size_t, PosHistory> leaf_index_cache(cache_limit);
    LruCache<size_t, Node> leaf_memo_cache(cache_limit);

    std::vector<LruCache<size_t, PosHistory>> node_index_caches(depth, LruCache<size_t, PosHistory>(cache_limit / 4));
    std::vector<LruCache<size_t, Node>> node_memo_caches(depth, LruCache<size_t, Node>(cache_limit / 4));

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
    size_t total_voxels = std::filesystem::file_size(filepath) / sizeof(VoxelData);
    size_t processed_voxels = 0;

    auto calculate_divergence = [](u64 morton_a, u64 morton_b) -> int
        {
            u64 diff = morton_a ^ morton_b;
            if (diff == 0) return 0;
            return (63 - std::countl_zero(diff)) / 3;
        };

    auto insert_leaf = [&](TempNode<Voxel>& leaves) -> Node
        {
            return insert_with_sliding_window(
                leaves,
                leaves_,
                leaf_index_cache,
                leaf_memo_cache
            );
        };

    auto insert_node = [&](TempNode<Node>& nodes, int level) -> Node
        {
            return insert_with_sliding_window(
                nodes,
                level_nodes[level],
                node_index_caches[level],
                node_memo_caches[level]
            );
        };

    if (!file.read(reinterpret_cast<char*>(&voxel_data), sizeof(VoxelData))) return Node{};
    processed_voxels++;

    u64 last_morton = voxel_data.morton;
    temp_leaf.add(last_morton & 0b111, voxel_data.voxel);

    while (file.read(reinterpret_cast<char*>(&voxel_data), sizeof(VoxelData)))
    {
        processed_voxels++;
        if (processed_voxels % 10000 == 0)
        {
            double percent = (static_cast<double>(processed_voxels) * 100.0) / total_voxels;
            std::print("\rProcessed voxels: {}/{} [{:.2f}%]", processed_voxels, total_voxels, percent);
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

                if (current_level >= depth) break;

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