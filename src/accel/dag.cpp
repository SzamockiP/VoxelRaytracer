#include <vrt/accel/dag.hpp>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <bit>
#include <ranges>
#include <print>
#include <string_view>
#include <unordered_map>

// return index to the first object of the sequence
vrt::Dag::Node vrt::Dag::add_node(const std::array<Node, 8>& node, u8 mask)
{
    // function does not work for empty node
    if (mask == 0)
    {
        throw std::runtime_error("dag.cpp::add_node::mask");
    }

    //// check, if all of nodes are the same and are leaves to deduplicate 
    //if (mask == 0xFF && node[0].is_leaf() && std::ranges::all_of(node, [&node](const auto& x) { return x == node[0]; }))
    //{
    //    // we just return the node, and delegate insertion to next level
    //    return node[0];
    //}

    // gather nodes before sort
    std::array<std::pair<Node, u8>, 8> active_nodes;
    int active_count = 0;
    for (u8 i = 0; i < 8; i++)
    {
        if (mask & (1u << i))
        {
            active_nodes[active_count++] = { node[i], i };
        }
    }

    // sort array by first node values
    std::sort(active_nodes.begin(), active_nodes.begin() + active_count,
        [](const auto& a, const auto& b) { return a.first.raw < b.first.raw; });

    // generate sequence and descriptor
    std::array<Node, 8> unique_nodes;
    u32 unique_count = 0;
    u32 descriptor = 0;

    for (int i = 0; i < active_count; i++)
    {
        // compress leaves
        if (i == 0 || active_nodes[i].first.raw != unique_nodes[unique_count - 1].raw)
        {
            unique_nodes[unique_count] = active_nodes[i].first;
            unique_count++;
        }

        u8 quarter_size = unique_count - 1;
        u8 original_octant = active_nodes[i].second;

        u32 child_data = 0b1000u | quarter_size;
        descriptor |= (child_data << (4 * original_octant));
    }

    // insert sequence into leaves_ and leaf_indices_
    u32 index = insert_node(std::span{ unique_nodes }.first(unique_count));

    // create node with sequence index and descriptor
    Node new_node;
    new_node.index = index;
    new_node.descriptor = descriptor;

    // return node
    return new_node;
};

// return index to the first object of the sequence
vrt::Dag::Node vrt::Dag::add_leaf(const std::array<Voxel, 8>& leaf, u8 mask)
{
	// function does not work for empty leaf
    if (mask == 0)
    {
        throw std::runtime_error("dag.cpp::add_leaf::mask");
    }

    // check, if you can make node a whole leaf
    if (mask == 0xFF && std::ranges::all_of(leaf, [&leaf](const auto& x) { return x == leaf[0]; }))
    {
        Node solid_leaf;
        solid_leaf.index = 1u << 31;
        solid_leaf.voxel = leaf[0];
        return solid_leaf;
    }

    // gather leaves before sort
    std::array<std::pair<Voxel, u8>, 8> active_leaves;
    int active_count = 0;
    for (u8 i = 0; i < 8; i++)
    {
        if (mask & (1u << i))
        {
            active_leaves[active_count++] = { leaf[i], i };
        }
    }

    // sort array by first leaf values
    std::sort(active_leaves.begin(), active_leaves.begin() + active_count,
        [](const auto& a, const auto& b){ return a.first.rgbe < b.first.rgbe; });


    // generate sequence and descriptor
    std::array<Voxel, 8> unique_leaves;
    u32 unique_count = 0;
    u32 descriptor = 0;

    for (int i = 0; i < active_count; i++)
    {
        // compress leaves
        if (i == 0 || active_leaves[i].first.rgbe != unique_leaves[unique_count - 1].rgbe)
        {
            unique_leaves[unique_count] = active_leaves[i].first;
            unique_count++;
        }

        u8 quarter_size = unique_count - 1;
        u8 original_octant = active_leaves[i].second;

        u32 child_data = 0b1000u | quarter_size;
        descriptor |= (child_data << (4 * original_octant));
    }


    // insert sequence into leaves_ and leaf_indices_
    u32 index = insert_leaf(std::span{ unique_leaves }.first(unique_count));

    // create node with sequence index and descriptor
    Node node;
    node.index = index;
    node.descriptor = descriptor;
    
    // return node
    return node;
};

vrt::u64 hash_voxel_sequence(std::span<vrt::Dag::Voxel> voxel_sequence)
{
    vrt::u64 hash = 0xcbf29ce484222325;
    for (const auto voxel : voxel_sequence)
    {
        for (int i = 0; i < 32; i += 8)
        {
            hash ^= (voxel.rgbe >> i) & 0xFF;
            hash *= 0x00000100000001b3;
        }
    }
    return hash;
}

vrt::u32 vrt::Dag::insert_leaf(std::span<Voxel> voxel_sequence)
{
    const u32 K = voxel_sequence.size();
    u64 hash = hash_voxel_sequence(voxel_sequence);

    // check if sequence exists
    auto range = leaf_indices_.equal_range(hash);
    auto vec_span = std::span{ leaves_ };

    for (auto it = range.first; it != range.second; ++it)
    {
        if (it->second + K > vec_span.size()) continue;
        auto vec_subspan = vec_span.subspan(it->second, K);
        if (std::ranges::equal(voxel_sequence, vec_subspan))
        {
            return it->second;
        }
    }

    // didn't found
    int max_overlap = std::min<int>(K - 1, vec_span.size());
    int overlap_size = 0;

    // check for overlaps on end
    for (int n = max_overlap; n > 0; n--)
    {
        if (std::ranges::equal(vec_span.last(n), voxel_sequence.first(n)))
        {
            overlap_size = n;
            break;
        }
    }

    u32 return_index = leaves_.size() - overlap_size;
    u32 old_size = leaves_.size();

    leaves_.insert(leaves_.end(), voxel_sequence.begin() + overlap_size, voxel_sequence.end());
    u32 new_size = leaves_.size();

    for (u32 end_idx = old_size; end_idx < new_size; end_idx++)
    {
        // how far back can we look
        u32 max_backtrack = std::min<u32>(8, end_idx + 1);

        // build and hash sequences ending at end_idx with growing lengths
        for (u32 seq_len = 1; seq_len <= max_backtrack; seq_len++)
        {
            u32 start_idx = end_idx - seq_len + 1;

            // extract the exact sub_sequence and calculate its hash
            auto sub_sequence = std::span{ leaves_ }.subspan(start_idx, seq_len);
            u64 hash = hash_voxel_sequence(sub_sequence);

            leaf_indices_.insert({ hash, start_idx });
        }
    }

    return return_index;
}

vrt::u64 hash_node_sequence(std::span<vrt::Dag::Node> node_sequence)
{
    vrt::u64 hash = 0xcbf29ce484222325;
    for (const auto node : node_sequence)
    {
        for (int i = 0; i < 64; i += 8)
        {
            hash ^= (node.raw >> i) & 0xFF;
            hash *= 0x00000100000001b3;
        }
    }
    return hash;
}

vrt::u32 vrt::Dag::insert_node(std::span<Node> node_sequence)
{
    const u32 K = node_sequence.size();
    u64 hash = hash_node_sequence(node_sequence);

    // check if sequence exists
    auto range = node_indices_.equal_range(hash);
    auto vec_span = std::span{ nodes_ };

    for (auto it = range.first; it != range.second; ++it)
    {
        if (it->second + K > vec_span.size()) continue;
        auto vec_subspan = vec_span.subspan(it->second, K);
        if (std::ranges::equal(node_sequence, vec_subspan))
        {
            return it->second;
        }
    }

    // didn't found
    int max_overlap = std::min<int>(K - 1, vec_span.size());
    int overlap_size = 0;

    // check for overlaps on end
    for (int n = max_overlap; n > 0; n--)
    {
        if (std::ranges::equal(vec_span.last(n), node_sequence.first(n)))
        {
            overlap_size = n;
            break;
        }
    }

    u32 return_index = nodes_.size() - overlap_size;
    u32 old_size = nodes_.size();

    nodes_.insert(nodes_.end(), node_sequence.begin() + overlap_size, node_sequence.end());
    u32 new_size = nodes_.size();

    for (u32 end_idx = old_size; end_idx < new_size; end_idx++)
    {
        // how far back can we look
        u32 max_backtrack = std::min<u32>(8, end_idx + 1);

        // build and hash sequences ending at end_idx with growing lengths
        for (u32 seq_len = 1; seq_len <= max_backtrack; seq_len++)
        {
            u32 start_idx = end_idx - seq_len + 1;

            // extract the exact sub_sequence and calculate its hash
            auto sub_sequence = std::span{ nodes_ }.subspan(start_idx, seq_len);
            u64 hash = hash_node_sequence(sub_sequence);

            node_indices_.insert({ hash, start_idx });
        }
    }

    return return_index;
}

static vrt::u32 shape_terrain(const glm::vec3& pos)
{
    float surface_height = std::sin(pos.x * 0.05f) * std::cos(pos.z * 0.05f) * 20.0f;

    return pos.y <= surface_height ? 1 : 0;
}

static vrt::u32 shape_gyroid(const glm::vec3& pos)
{
    glm::vec3 p = pos * 0.5f;


    float val = std::sin(p.x) * std::cos(p.y) +
        std::sin(p.y) * std::cos(p.z) +
        std::sin(p.z) * std::cos(p.x);

    return (val > -0.3f && val < 0.3f) ? 1 : 0;
}

static vrt::u32 shape_solid_sphere(const glm::vec3& pos)
{
    const float radius = 80.0f;
    return glm::length(pos) <= radius ? 1 : 0;
}

static std::optional<vrt::Dag::Voxel> get_voxel(glm::vec3 pos)
{
    if (shape_gyroid(pos) == 1)
    {
        // 1. Zabezpieczenie przed (0,0,0), żeby normalize() nie zwróciło NaN
        glm::vec3 n = (pos == glm::vec3(0.0f)) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::normalize(pos);

        // 2. Mapowanie wektora z zakresu [-1.0, 1.0] na kolory [0, 255]
        vrt::u8 r = static_cast<vrt::u8>((n.x + 1.0f) * 0.5f * 255.0f);
        vrt::u8 b = static_cast<vrt::u8>((n.y + 1.0f) * 0.5f * 255.0f); // Oś Y jako niebieski (Twoje życzenie)
        vrt::u8 g = static_cast<vrt::u8>((n.z + 1.0f) * 0.5f * 255.0f); // Oś Z jako zielony (Twoje życzenie)

        // 3. Wpisujemy do unii z wykorzystaniem struktury RGBE
        vrt::Dag::Voxel v;
        v.r = r;
        v.g = g;
        v.b = b;
        v.e = 0; // Kanał dodatkowy/Alpha ustawiamy na 0

        return v;
    }
    return std::nullopt;
}

std::optional<vrt::Dag::Node> vrt::Dag::build(u8 depth, const glm::vec3& center, const std::function<std::optional<Voxel>(glm::vec3)>& sampler)
{
    u32 half_size = 1u << depth;
    float offset = half_size * 0.5f;

    const glm::vec3 offsets[8] = {
        {-offset, -offset, -offset },
        { offset, -offset, -offset },
        {-offset,  offset, -offset },
        { offset,  offset, -offset },
        {-offset, -offset,  offset },
        { offset, -offset,  offset },
        {-offset,  offset,  offset },
        { offset,  offset,  offset }
    };

    if (depth == 0)
    {
        std::array<Voxel, 8> leaves;
        u8 mask = 0;

        for (int i = 0; i < 8; i++)
        {
            glm::vec3 pos = center + offsets[i];

            // TUTAJ ZMIANA: Używamy wstrzykniętej funkcji zamiast sztywnego get_voxel()
            std::optional<Voxel> opt_voxel = sampler(pos);

            if (opt_voxel.has_value())
            {
                leaves[i] = opt_voxel.value();
                mask |= (1u << i);
            }
        }

        if (mask == 0) return std::nullopt;

        return add_leaf(leaves, mask);
    }
    else
    {
        std::array<Node, 8> children;
        u8 mask = 0;
        for (int i = 0; i < 8; ++i)
        {
            glm::vec3 pos = center + offsets[i];

            // TUTAJ ZMIANA: Przekazujemy sampler oczko niżej do kolejnego wywołania rekurencyjnego!
            std::optional<Node> opt_child = build(depth - 1, pos, sampler);

            if (opt_child.has_value())
            {
                children[i] = opt_child.value();
                mask |= (1u << i);
            }
        }

        if (mask == 0) return std::nullopt;

        return add_node(children, mask);
    }
}

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


template <typename T>
vrt::Dag::Node insert_with_sliding_window(
    const TempNode<T>& temp,
    std::vector<T>& target_buffer,
    std::unordered_map<size_t, int>& index_map)
{
    vrt::Dag::Node final_node;
    final_node.index = 0;
    final_node.descriptor = 0;

    int N = temp.elements.size();
    if (N == 0) return final_node;

    bool found_match = false;
    int best_index = -1;
    int best_offsets[8] = { 0 }; // offset mask for node

    // looking in window
    // 1. SZYBKIE SZUKANIE W OZNACZONYM OKNIE (O(1) zamiast O(N))
    size_t first_element_hash = hash_memory(temp.elements[0]);
    auto it = index_map.find(first_element_hash);

    if (it != index_map.end())
    {
        // Wyciągamy naszą jedyną, najnowszą pozycję pierwszego elementu
        int pos = it->second;

        // Okno zawierające nasz element na pozycji 'pos' 
        // mogło zacząć się maksymalnie 7 pozycji wcześniej
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
                if (!found_e)
                {
                    window_has_all = false;
                    break;
                }
            }

            if (window_has_all)
            {
                found_match = true;
                best_index = i; // Zmieniłem z best_index na best_base_index, tak jak miałeś pierwotnie
                for (int e = 0; e < N; ++e) best_offsets[e] = current_offsets[e];
                break; // Znaleźliśmy okno, przerywamy pętlę 'i'
            }
        }
    }

    // look for overlap on end
    if (!found_match)
    {
        int max_overlap = std::min(7, (int)target_buffer.size());

        for (int overlap = max_overlap; overlap >= 1; --overlap)
        {
            int prospective_base = target_buffer.size() - overlap;

            std::vector<T> missing_elements;
            int prospective_offsets[8] = { 0 };
            bool element_resolved[8] = { false };

            // get elements already at the tail 
            for (int e = 0; e < N; ++e)
            {
                for (int w = 0; w < overlap; ++w)
                {
                    if (temp.elements[e] == target_buffer[prospective_base + w])
                    {
                        prospective_offsets[e] = w;
                        element_resolved[e] = true;
                        break;
                    }
                }
            }

            // get missing elements
            for (int e = 0; e < N; ++e)
            {
                if (!element_resolved[e])
                {
                    bool already_missing = false;
                    for (size_t m = 0; m < missing_elements.size(); ++m)
                    {
                        if (temp.elements[e] == missing_elements[m])
                        {
                            // set offset for missing element
                            prospective_offsets[e] = overlap + m;
                            already_missing = true;
                            break;
                        }
                    }
                    if (!already_missing)
                    {
                        prospective_offsets[e] = overlap + missing_elements.size();
                        missing_elements.push_back(temp.elements[e]);
                    }
                }
            }

            // check, if they fit
            if (overlap + missing_elements.size() <= 8)
            {
                found_match = true;
                best_index = prospective_base;
                for (int e = 0; e < N; ++e) best_offsets[e] = prospective_offsets[e];

                for (const auto& missing : missing_elements)
                {
                    target_buffer.push_back(missing);
                    index_map[hash_memory(missing)] = target_buffer.size() - 1;
                }
                break;
            }
        }
    }

    // no window with partial overlap
    if (!found_match)
    {
        best_index = target_buffer.size();
        std::vector<T> unique_elements;

        for (int e = 0; e < N; ++e)
        {
            int offset = -1;
            // make unique elements
            for (size_t u = 0; u < unique_elements.size(); ++u)
            {
                if (temp.elements[e] == unique_elements[u])
                {
                    offset = u;
                    break;
                }
            }
            
            if (offset == -1)
            {
                offset = unique_elements.size();
                unique_elements.push_back(temp.elements[e]);
                target_buffer.push_back(temp.elements[e]);
                index_map[hash_memory(temp.elements[e])] = target_buffer.size() - 1;
            }
            best_offsets[e] = offset;
        }
    }

    // build descriptor
    final_node.index = best_index;
    int e_idx = 0;

    // iterate over every octant and set offset
    for (int octant = 0; octant < 8; ++octant)
    {
        if (temp.mask & (1 << octant))
        {
            final_node.set_child_offset(octant, best_offsets[e_idx]);
            e_idx++;
        }
    }

    return final_node;
}

vrt::Dag::Node vrt::Dag::build(u8 depth, const std::filesystem::path& filepath)
{
    if (!std::filesystem::exists(filepath))
    {
        throw std::runtime_error("Error: vrt::Dag::build filepath doesn't exist.");
    }

    std::ifstream file(filepath, std::ios::binary);

    
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

    std::unordered_map<size_t, int> leaf_index_map;
    std::vector<std::unordered_map<size_t, int>> node_index_maps(depth);
    auto insert_leaf = [&](TempNode<Voxel>& leaves) -> Node
        {
            return insert_with_sliding_window(leaves, leaves_, leaf_index_map);
        };

    auto insert_node = [&](TempNode<Node>& nodes, int level) -> Node
        {
            return insert_with_sliding_window(nodes, level_nodes[level], node_index_maps[level]);
        };

    // get first voxel, may be empty so return empty root
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
            std::print("\rProcessed voxels: {}/{} [{:.2f}%]", processed_voxels,total_voxels,percent);
        }
        // voxel in the same node
        if ((last_morton >> 3) == (voxel_data.morton >> 3))
        {
            temp_leaf.add(voxel_data.morton & 0b111, voxel_data.voxel);
        }
        else
        {
            // a) Zwijamy liście do węzła Poziomu 0
            Node node = insert_leaf(temp_leaf);
            temp_leaf.clear();

            int current_level = 0;
            temp_nodes[current_level].add((last_morton >> 3) & 0b111, node);

            // b) Obliczamy, na którym poziomie ścieżki się rozchodzą
            int max_level = calculate_divergence(last_morton, voxel_data.morton);

            // ZABEZPIECZENIE: Ucinamy bzdury, jeśli w kodach Mortona są śmieci 
            // (Rozgałęzienie nie może być wyższe niż przedostatni poziom drzewa)
            max_level = std::min(max_level, (int)depth - 1);

            // c) Kaskada - UWAGA: max_level - 1 ! (Nie zamykamy rodzica, który nadal przyjmuje dzieci)
            while (current_level < max_level - 1)
            {
                node = insert_node(temp_nodes[current_level], current_level);
                temp_nodes[current_level].clear();

                ++current_level;

                // Zabezpieczenie ostateczne (Nigdy nie wpychamy ponad zdefiniowaną głębokość)
                if (current_level >= depth) break;

                // POPRAWNA MATEMATYKA (Z nawiasami!)
                u8 parent_octant = (last_morton >> (3 * (current_level + 1))) & 0b111;
                temp_nodes[current_level].add(parent_octant, node);
            }

            temp_leaf.add(voxel_data.morton & 0b111, voxel_data.voxel);
        }

        last_morton = voxel_data.morton;
    }

    // flush rest of tree
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

            // if not root, pass higher
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

    // 2. ALOKACJA OSTATECZNEGO BUFORA TOPOLOGII
    std::vector<Node> gpu_topology_buffer;
    nodes_.reserve(total_topology_nodes);

    // 3. KOPIOWANIE I ŁATANIE WSKAŹNIKÓW (The Magic)
    for (int d = 0; d < depth; ++d)
    {
        for (const Node& node : level_nodes[d])
        {
            Node patched_node = node;

            // Łatamy wskaźnik TYLKO dla węzłów powyżej poziomu 0.
            // (Bo poziom 0 wskazuje na final_leaves, które są oddzielnym buforem)
            if (d > 0)
            {
                // Dodajemy globalny offset poziomu, na który ten węzeł wskazuje (czyli d - 1)
                patched_node.index += level_offsets[d - 1];
            }

            nodes_.push_back(patched_node);
        }
    }

    return nodes_.empty() ? Node{} : nodes_.back();
}