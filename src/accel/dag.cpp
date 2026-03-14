#include <vrt/accel/dag.hpp>
#include <algorithm>
#include <stdexcept>

// return index to the first object of the sequence
vrt::Dag::Node vrt::Dag::add_node(const std::array<Node, 8>& node, u8 mask)
{
    // function does not work for empty node
    if (mask == 0)
    {
        throw std::runtime_error("dag.cpp::add_node::mask");
    }

    // check, if all of nodes are the same and are leaves to deduplicate 
    if (mask == 0xFF && (node[0].index & (1u << 31)) != 0 && std::ranges::all_of(node, [&node](const auto& x) { return x == node[0]; }))
    {
        // we just return the node, and delegate insertion to next level
        return node[0];
    }

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
        if (i == 0 || active_nodes[i].first.raw != active_nodes[unique_count - 1].first.raw)
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

std::optional<vrt::Dag::Voxel> get_voxel(glm::vec3 pos)
{
    glm::vec3 p = pos * 0.5f;

    float val = std::sin(p.x) * std::cos(p.y) +
        std::sin(p.y) * std::cos(p.z) +
        std::sin(p.z) * std::cos(p.x);

    if (val > -0.3f && val < 0.3f)
    {
        return vrt::Dag::Voxel{ .rgbe = 0xFFFFFF00 };
    }

    return std::nullopt;
}

std::optional<vrt::Dag::Node> vrt::Dag::build(u8 depth, const glm::vec3& center)
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

            // custom shape function
            std::optional<Voxel> opt_voxel = get_voxel(pos);

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

            std::optional<Node> opt_child = build(depth - 1, pos);

            if (opt_child.has_value())
            {
                children[i] = opt_child.value();
                mask |= (1u << i);
            }
        }

        if (mask == 0) return std::nullopt;

        return add_node(children, mask);
    }
};


