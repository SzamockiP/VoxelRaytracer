#include <vrt/accel/tlas.hpp>
#include <algorithm>

void vrt::Tlas::build(const std::vector<Instance>& instances) 
{
	compute_morton_entries(instances);

	std::ranges::sort(morton_entries_, [](const MortonEntry& a, const MortonEntry& b) 
	{
		return a.code < b.code;
	});
}

void vrt::Tlas::compute_morton_entries(const std::vector<Instance>& instances)
{
	if (instances.empty()) return;

	morton_entries_.clear();
	morton_entries_.reserve(instances.size());


	AABB bounds{};
	for (const auto& instance : instances) {
		float size = 1u << instance.depth;
		bounds.grow(instance.transform[3]);
	}

	glm::vec3 bounds_size{ bounds.max - bounds.min };

	if (bounds_size.x == 0.0f) bounds_size.x = 1.0f;
	if (bounds_size.y == 0.0f) bounds_size.y = 1.0f;
	if (bounds_size.z == 0.0f) bounds_size.z = 1.0f;

	auto expand_bits_10 = [](u32 v) -> u32 {
		v = (v | (v << 16)) & 0x030000FF;
		v = (v | (v << 8)) & 0x0300F00F;
		v = (v | (v << 4)) & 0x030C30C3;
		v = (v | (v << 2)) & 0x09249249;
		return v;
	};

	for (u32 i = 0; i < instances.size(); i++) {
		glm::vec3 norm_pos = (glm::vec3(instances[i].transform[3]) - bounds.min) / bounds_size;
		glm::ivec3 scaled_pos = glm::clamp(norm_pos * 1024.0f, 0.0f, 1023.0f);

		u32 x = expand_bits_10(static_cast<u32>(scaled_pos.x));
		u32 y = expand_bits_10(static_cast<u32>(scaled_pos.y));
		u32 z = expand_bits_10(static_cast<u32>(scaled_pos.z));

		u32 morton_code = x | (y << 1) | (z << 2);
		morton_entries_.push_back({ .index = i, .code = morton_code });
	} 
}