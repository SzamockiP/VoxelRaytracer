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

            Hit intersect(const Ray& ray, u8 depth, u32 root_index) const noexcept;

            u32 build(u8 depth, const std::filesystem::path& filepath);


            // Rozmiar samej topologii (geometrii i wskaźników) w bajtach
            size_t topology_size_bytes() const
            {
                return nodes_.size() * sizeof(u32);
            }

            // Całkowity rozmiar struktury w pamięci (w bajtach)
            size_t total_size_bytes() const
            {
                return topology_size_bytes();
            }

            // Liczba unikalnych węzłów topologicznych
            size_t num_nodes() const
            {
                size_t count = 0;
                size_t i = 0;
                // Skaczemy po tablicy dokładnie tak samo jak w fazie kompresji
                while (i < nodes_.size())
                {
                    count++;
                    u32 desc = nodes_[i];
                    u32 child_count = (desc == 0) ? 0 : (max_offset(desc) + 1);
                    i += 1 + child_count; // Skok o deskryptor + jego wskaźniki
                }
                return count;
            }

            void print_stats() const
            {
                double topo_mb = topology_size_bytes() / (1024.0 * 1024.0);
                double total_mb = total_size_bytes() / (1024.0 * 1024.0);

                std::printf("\n================ SVDAG V2.0 STATS ================\n");
                std::printf("Unique Nodes : %zu\n", num_nodes());
                std::printf("Topology Size: %zu B (%.2f MB)\n", topology_size_bytes(), topo_mb);
                std::printf("Total Size   : %zu B (%.2f MB)\n", total_size_bytes(), total_mb);
                std::printf("==================================================\n\n");
            }

        private:
            std::vector<u32> nodes_;

            static u32 pmax(u32 x, u32 y)
            {
                u32 m_top = ((x | 0x88888888) - y) & 0x88888888;
                u32 m = m_top | (m_top - (m_top >> 3));
                return (x & m) | (y & ~m);
            }

            static u8 max_offset(u32 val)
            {

                u8 max_off = 0;
                // Sprawdzamy wszystkie 8 oktantów w deskryptorze
                for (int i = 0; i < 8; ++i)
                {
                    u32 chunk = (val >> (i * 4)) & 0xF;
                    if (chunk & 0b1000) // Jeśli Valid Bit jest zapalony
                    {
                        u8 offset = chunk & 0b0111;
                        if (offset > max_off)
                        {
                            max_off = offset;
                        }
                    }
                }
                return max_off;

                /*val = pmax(val, val >> 16);
                val = pmax(val, val >> 8);
                val = pmax(val, val >> 4);
                return val & 0b0111;*/
            }
        };
	}
}