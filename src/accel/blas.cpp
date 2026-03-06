#include <vrt/accel/blas.hpp>
#include <vrt/voxel/voxel.hpp>
#include <glm/glm.hpp>
#include <algorithm>

static vrt::u32 shape_gyroid(const glm::vec3& pos)
{
	glm::vec3 p = pos * 0.5f;


	float val = std::sin(p.x) * std::cos(p.y) +
				std::sin(p.y) * std::cos(p.z) +
				std::sin(p.z) * std::cos(p.x);

	return (val > -0.3f && val < 0.3f) ? 1 : 0;
}

static vrt::u32 shape_torus(const glm::vec3& pos)
{
	const float major_radius = 8.0f;
	const float minor_radius = 4.0f;

	float q = std::sqrt(pos.x * pos.x + pos.z * pos.z) - major_radius;

	float distance = std::sqrt(q * q + pos.y * pos.y);

	return distance <= minor_radius ? 1 : 0;
}

static vrt::Voxel shape(const glm::vec3& pos)
{
	return shape_gyroid(pos) == 1 ? vrt::Voxel::FULL : vrt::Voxel::EMPTY;
}

vrt::u32 vrt::Blas::build(glm::vec3 center, u8 depth)
{

	if (depth == 1)
	{
		Leaf l{
			shape(center + glm::vec3{ -0.5f, -0.5f, -0.5f }),
			shape(center + glm::vec3{  0.5f, -0.5f, -0.5f }),
			shape(center + glm::vec3{ -0.5f,  0.5f, -0.5f }),
			shape(center + glm::vec3{  0.5f,  0.5f, -0.5f }),
			shape(center + glm::vec3{ -0.5f, -0.5f,  0.5f }),
			shape(center + glm::vec3{  0.5f, -0.5f,  0.5f }),
			shape(center + glm::vec3{ -0.5f,  0.5f,  0.5f }),
			shape(center + glm::vec3{  0.5f,  0.5f,  0.5f })
		};

		return add_leaf(l);
	}
	else
	{
		depth--;
		const float offset = static_cast<float>(1u << depth) * 0.5f;
		Node n = {
			build(center + glm::vec3{ -offset, -offset, -offset }, depth),
			build(center + glm::vec3{  offset, -offset, -offset }, depth),
			build(center + glm::vec3{ -offset,  offset, -offset }, depth),
			build(center + glm::vec3{  offset,  offset, -offset }, depth),
			build(center + glm::vec3{ -offset, -offset,  offset }, depth),
			build(center + glm::vec3{  offset, -offset,  offset }, depth),
			build(center + glm::vec3{ -offset,  offset,  offset }, depth),
			build(center + glm::vec3{  offset,  offset,  offset }, depth)
		};
		return add_node(n);
	}
}

vrt::u32 vrt::Blas::add_node(const Node& node)
{
	// check if empty
	if (std::ranges::all_of(node.indices, [](auto x) { return x == EMPTY; }))
	{
		return EMPTY;
	}

	auto kv = node_indices_.find(node);
	// exists
	if (kv != node_indices_.end())
	{
		auto idx = kv->second;
		node_refcounts_[idx]++;
		return idx;
	}
	// does not exist
	else
	{
		u32 idx;
		if (free_node_indices_.size() > 0)
		{
			idx = free_node_indices_.back();
			free_node_indices_.pop_back();
			nodes_[idx] = node;
			node_refcounts_[idx] = 1;
		}
		else
		{
			idx = nodes_.size();
			nodes_.push_back(node);
			node_refcounts_.push_back(1);
		}
		node_indices_[node] = idx;
		return idx;
	}
}

vrt::u32 vrt::Blas::add_leaf(const Leaf& leaf)
{
	// check if empty
	if (std::ranges::all_of(leaf.voxels, [](auto x) { return x == Voxel::EMPTY; }))
	{
		return EMPTY;
	}

	auto kv = leaf_indices_.find(leaf);
	// exists
	if (kv != leaf_indices_.end())
	{
		auto idx = kv->second;
		leaf_refcounts_[idx]++;
		return idx;
	}
	// does not exist
	else
	{
		vrt::u32 idx;
		if (free_leaf_indices_.size() > 0)
		{
			idx = free_leaf_indices_.back();
			free_leaf_indices_.pop_back();
			leaves_[idx] = leaf;
			leaf_refcounts_[idx] = 1;
		}
		else
		{
			idx = leaves_.size();
			leaves_.push_back(leaf);
			leaf_refcounts_.push_back(1);
		}
		leaf_indices_[leaf] = idx;
		return idx;
	}
}

std::size_t vrt::Blas::NodeHasher::operator()(const Node& node) const noexcept
{
	std::size_t hash = 0;
	for (auto index : node.indices)
	{
		std::size_t h = std::hash<vrt::u32>{}(index);
		hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
	}
	return hash;
}

std::size_t vrt::Blas::LeafHasher::operator()(const Leaf& leaf) const noexcept
{
	std::size_t hash = 0;
	for (auto voxel : leaf.voxels)
	{
		std::size_t h = std::hash<vrt::u32>{}(static_cast<vrt::u32>(voxel));
		hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
	}
	return hash;
}