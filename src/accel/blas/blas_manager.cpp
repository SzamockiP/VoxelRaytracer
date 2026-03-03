#include <vrt/accel/blas/blas_manager.hpp>
#include <algorithm>

std::uint32_t vrt::BlasManager::AddNode(const Node& node)
{
	// check if empty
	if (std::ranges::all_of(node.indices, [](auto x) { return x == EMPTY; }))
	{
		return EMPTY;
	}

	auto kv = node_indices.find(node);
	// exists
	if (kv != node_indices.end())
	{
		auto idx = kv->second;
		node_refcounts[idx]++;
		return idx;
	}
	// does not exist
	else
	{
		std::uint32_t idx;
		if (free_node_indices_.size() > 0)
		{
			idx = free_node_indices_.back();
			free_node_indices_.pop_back();
			dag_pool_.nodes[idx] = node;
			node_refcounts[idx] = 1;
		}
		else
		{
			idx = dag_pool_.nodes.size();
			dag_pool_.nodes.push_back(node);
			node_refcounts.push_back(1);
		}
		node_indices[node] = idx;
		return idx;
	}
}

std::uint32_t vrt::BlasManager::AddLeaf(const Leaf& leaf)
{
	// check if empty
	if (std::ranges::all_of(leaf.voxels, [](auto x) { return x == Voxel::EMPTY; }))
	{
		return EMPTY;
	}

	auto kv = leaf_indices.find(leaf);
	// exists
	if (kv != leaf_indices.end())
	{
		auto idx = kv->second;
		leaf_refcounts[idx]++;
		return idx;
	}
	// does not exist
	else
	{
		std::uint32_t idx;
		if (free_leaf_indices_.size() > 0)
		{
			idx = free_leaf_indices_.back();
			free_leaf_indices_.pop_back();
			dag_pool_.leaves[idx] = leaf;
			leaf_refcounts[idx] = 1;
		}
		else
		{
			idx = dag_pool_.leaves.size();
			dag_pool_.leaves.push_back(leaf);
			leaf_refcounts.push_back(1);
		}
		leaf_indices[leaf] = idx;
		return idx;
	}
}


void vrt::BlasManager::RemoveNode(std::uint32_t index)
{

}