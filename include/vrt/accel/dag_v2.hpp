#include <vector>
#include <filesystem>

#include <vrt/core/types.hpp>
#include <vrt/math/ray.hpp>

namespace vrt
{
	namespace v2
	{
        class Dag
        {
        public:
            union Voxel
            {
                u32 rgbe;
                struct
                {
                    u8 e;
                    u8 b;
                    u8 g;
                    u8 r;
                };

                bool operator==(const Voxel& other) const
                {
                    return rgbe == other.rgbe;
                };
            };

            struct Node
            {
                u32 descriptor{ 0 };

                bool octant_valid(u8 octant) const
                {
                    return (descriptor >> (octant * 4)) & 0b1000;
                }

                u32 child_offset(u8 octant) const
                {
                    return (descriptor >> (octant * 4)) & 0b0111;
                }

                void set_child(u8 octant, bool valid, u8 offset)
                {
                    descriptor &= ~(0b1111 << (octant * 4));

                    if (valid)
                    {
                        u8 data = (0b1000 | (offset & 0b0111));
                        descriptor |= (data << (octant * 4));
                    }
                }

                u8 child_count() const
                {

                    if (descriptor == 0) return 0;
                    return max_offset(descriptor) + 1;
                }
            };

            struct Hit
            {
                float t;
                glm::vec3 normal;
                Voxel voxel;
            };


            Node node_view(u32 index) const
            {
                return { .descriptor = nodes_[index] };
            }

            u32 child_index(u32 index, u8 octant) const
            {
                Node n = node_view(index);
                u8 offset = n.child_offset(offset);

                return nodes_[index + offset + 1];
            }

            Hit intersect(const Ray& ray, u8 depth, Node root) const noexcept;

            Node build(u8 depth, const std::filesystem::path& filepath);

        private:
            std::vector<u32> nodes_;
            std::vector<u32> leaves_;

            static u32 pmax(u32 x, u32 y)
            {
                u32 m_top = ((x | 0x88888888) - y) & 0x88888888;
                u32 m = m_top | (m_top - (m_top >> 3));
                return (x & m) | (y & ~m);
            }

            static u8 max_offset(u32 val)
            {
                val = pmax(val, val >> 16);
                val = pmax(val, val >> 8);
                val = pmax(val, val >> 4);
                return val & 0b0111;
            }
        };
	}
}