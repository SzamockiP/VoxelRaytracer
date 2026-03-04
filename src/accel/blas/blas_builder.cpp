#include <vrt/accel/blas/blas_builder.hpp>

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

vrt::u32 vrt::BlasBuilder::BuildTree(glm::vec3 center, u32 size)
{

	if (size == 2)
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

		return blas_manager_.AddLeaf(l);
	}
	else
	{
		u32 half_size = size >> 1;
		const float offset = static_cast<float>(size) * 0.25f;
		Node n = {
			BuildTree(center + glm::vec3{ -offset, -offset, -offset }, half_size),
			BuildTree(center + glm::vec3{  offset, -offset, -offset }, half_size),
			BuildTree(center + glm::vec3{ -offset,  offset, -offset }, half_size),
			BuildTree(center + glm::vec3{  offset,  offset, -offset }, half_size),
			BuildTree(center + glm::vec3{ -offset, -offset,  offset }, half_size),
			BuildTree(center + glm::vec3{  offset, -offset,  offset }, half_size),
			BuildTree(center + glm::vec3{ -offset,  offset,  offset }, half_size),
			BuildTree(center + glm::vec3{  offset,  offset,  offset }, half_size)
		};
		return blas_manager_.AddNode(n);
	}
}

