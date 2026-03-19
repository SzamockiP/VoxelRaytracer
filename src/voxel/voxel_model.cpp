#include <vrt/voxel/voxel_model.hpp>
#include <iostream>
#include <algorithm>

#define TINYOBJLOADER_DISABLE_FAST_FLOAT
#define TINYOBJLOADER_IMPLEMENTATION
#include <tol/tiny_obj_loader.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>
#include <map>

struct TextureData
{
    int width = 0, height = 0, channels = 0;
    unsigned char* data = nullptr;
};

// -----------------------------------------------------------------------------
// MATEMATYKA: Szybki test przecięcia AABB (Sześcianu) z Trójkątem (SAT)
// -----------------------------------------------------------------------------
static bool barycentricOverlap(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float& out_u, float& out_v, float& out_w)
{
    glm::vec3 v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;

    if (std::abs(denom) < 1e-8f) return false; // Ignorujemy zepsute trójkąty o zerowej powierzchni

    out_v = (d11 * d20 - d01 * d21) / denom;
    out_w = (d00 * d21 - d01 * d20) / denom;
    out_u = 1.0f - out_v - out_w;

    // Magiczny Epsilon (0.05f). To on łata luki między wokselami, lekko powiększając trójkąt!
    float eps = 0.05f;
    return (out_u >= -eps) && (out_v >= -eps) && (out_w >= -eps);
}

// -----------------------------------------------------------------------------
// IMPLEMENTACJA KLASY
// -----------------------------------------------------------------------------
bool vrt::VoxelModel::load_obj(const std::string& filepath, int expected_size)
{
    this->size = expected_size;
    tinyobj::ObjReaderConfig reader_config;
    tinyobj::ObjReader reader;

    std::cout << "Czytam plik .obj: " << filepath << "...\n";
    if (!reader.ParseFromFile(filepath, reader_config))
    {
        if (!reader.Error().empty()) std::cerr << "Blad TinyObj: " << reader.Error() << "\n";
        return false;
    }
    if (!reader.Warning().empty())
    {
        std::cout << "Ostrzezenie TinyObj: " << reader.Warning() << "\n";
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // 1. Wyciągamy ścieżkę do folderu, żeby wiedzieć, gdzie szukać obrazków (.jpg / .png)
    std::string base_dir = filepath.substr(0, filepath.find_last_of("/\\") + 1);

    // 2. Ładujemy tekstury do pamięci TYLKO na czas wokselizacji
    std::map<std::string, TextureData> texture_cache;
    for (const auto& mat : materials)
    {
        if (!mat.diffuse_texname.empty() && texture_cache.find(mat.diffuse_texname) == texture_cache.end())
        {
            std::string tex_path = base_dir + mat.diffuse_texname;
            TextureData tex;
            // Wymuszamy 3 kanały (RGB), olewamy przezroczystość dla uproszczenia
            tex.data = stbi_load(tex_path.c_str(), &tex.width, &tex.height, &tex.channels, 3);

            if (tex.data)
            {
                texture_cache[mat.diffuse_texname] = tex;
                std::cout << "Zaladowano teksture: " << mat.diffuse_texname << "\n";
            }
            else
            {
                std::cerr << "UWAGA: Nie udalo sie znalezc pliku tekstury: " << tex_path << "\n";
            }
        }
    }

    // 1. SZUKAMY ABSOLUTNYCH GRANIC MODELU W PRZESTRZENI
    glm::vec3 min_bounds(std::numeric_limits<float>::max());
    glm::vec3 max_bounds(std::numeric_limits<float>::lowest());

    for (size_t i = 0; i < attrib.vertices.size() / 3; i++)
    {
        glm::vec3 v(attrib.vertices[3 * i + 0], attrib.vertices[3 * i + 1], attrib.vertices[3 * i + 2]);
        min_bounds = glm::min(min_bounds, v);
        max_bounds = glm::max(max_bounds, v);
    }

    // 2. WYLICZAMY IDEALNA SKALE I PRZESUNIECIE NA SRODEK
    glm::vec3 model_size_real = max_bounds - min_bounds;
    float max_dim = std::max({ model_size_real.x, model_size_real.y, model_size_real.z });

    // Zostawiamy 5% marginesu, zeby model nie dotykal "scian" naszego swiata
    float target_dim = this->size * 0.95f;
    float scale = target_dim / max_dim;

    glm::vec3 center_real = (max_bounds + min_bounds) * 0.5f;
    glm::vec3 grid_center(this->size * 0.5f);

    // 3. WOKSELIZACJA (Rasteryzacja)
    grid.clear();
    // Zakładam, że Twoja unia Voxel inicjalizuje się zerami dla pustego miejsca
    size_t total_voxels = static_cast<size_t>(this->size) * this->size * this->size;
    grid.resize(total_voxels, vrt::Dag::Voxel{ .rgbe = 0 });
    int voxels_painted = 0;
    glm::vec3 half_voxel(0.5f); // Połowa sześcianu 1x1x1

    std::cout << "Wokselizuje model na siatke " << size << "^3...\n";

    for (size_t s = 0; s < shapes.size(); s++)
    {

        std::cout << "Kształt " << s + 1 << " z " << shapes.size() << "...\r";

        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

            // Pobieramy 3 wierzchołki trójkąta i ich współrzędne UV!
            glm::vec3 tri[3];
            glm::vec2 uv[3] = { {0,0}, {0,0}, {0,0} };
            bool has_uvs = true;

            for (size_t v = 0; v < 3; v++)
            {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                // Pozycja 3D
                tri[v] = glm::vec3(
                    attrib.vertices[3 * size_t(idx.vertex_index) + 0],
                    attrib.vertices[3 * size_t(idx.vertex_index) + 1],
                    attrib.vertices[3 * size_t(idx.vertex_index) + 2]
                );
                tri[v] = (tri[v] - center_real) * scale + grid_center;

                // Pozycja UV (Tekstury)
                if (idx.texcoord_index >= 0)
                {
                    uv[v] = glm::vec2(
                        attrib.texcoords[2 * size_t(idx.texcoord_index) + 0],
                        attrib.texcoords[2 * size_t(idx.texcoord_index) + 1]
                    );
                }
                else
                {
                    has_uvs = false; // Ten trójkąt jest "łysy"
                }
            }

            // Minimalny prostopadłościan obejmujący tylko ten jeden trójkąt
            glm::vec3 tri_min = glm::min(tri[0], glm::min(tri[1], tri[2]));
            glm::vec3 tri_max = glm::max(tri[0], glm::max(tri[1], tri[2]));

            int min_x = std::max(0, static_cast<int>(std::floor(tri_min.x)));
            int min_y = std::max(0, static_cast<int>(std::floor(tri_min.y)));
            int min_z = std::max(0, static_cast<int>(std::floor(tri_min.z)));

            int max_x = std::min(this->size - 1, static_cast<int>(std::ceil(tri_max.x)));
            int max_y = std::min(this->size - 1, static_cast<int>(std::ceil(tri_max.y)));
            int max_z = std::min(this->size - 1, static_cast<int>(std::ceil(tri_max.z)));

            // Wyciągamy kolor materiału z .mtl (jeśli istnieje)
            int mat_id = shapes[s].mesh.material_ids[f];
            uint8_t r = 255, g = 255, b = 255; // Domyslnie bialy
            if (mat_id >= 0 && mat_id < materials.size())
            {
                r = static_cast<uint8_t>(materials[mat_id].diffuse[0] * 255.0f);
                g = static_cast<uint8_t>(materials[mat_id].diffuse[1] * 255.0f);
                b = static_cast<uint8_t>(materials[mat_id].diffuse[2] * 255.0f);
            }

            // Skanujemy TYLKO te woksele, które są wewnątrz Bounding Boxa tego trójkąta
            glm::vec3 normal = glm::cross(tri[1] - tri[0], tri[2] - tri[0]);
            float len = glm::length(normal);
            if (len > 1e-8f)
            { // Jeśli trójkąt ma powierzchnię
                normal /= len; // Normalizujemy
                float d = -glm::dot(normal, tri[0]);

                // Skanujemy woksele wokół trójkąta
                for (int z = min_z; z <= max_z; z++)
                {
                    for (int y = min_y; y <= max_y; y++)
                    {
                        for (int x = min_x; x <= max_x; x++)
                        {

                            glm::vec3 voxel_center(x + 0.5f, y + 0.5f, z + 0.5f);

                            // 1. Czy środek woksela jest blisko płaszczyzny trójkąta? (0.866 to promien okręgu opisanego na sześcianie)
                            float dist_to_plane = std::abs(glm::dot(normal, voxel_center) + d);

                            if (dist_to_plane <= 0.866f)
                            {
                                // 2. Rzutujemy punkt prostopadle na płaszczyznę ściany
                                glm::vec3 projected_p = voxel_center - normal * (glm::dot(normal, voxel_center) + d);

                                // 3. Sprawdzamy, czy rzutowany punkt leży fizycznie wewnątrz trójkąta
                                float u, v, w;
                                if (barycentricOverlap(projected_p, tri[0], tri[1], tri[2], u, v, w))
                                {
                                    size_t flat_idx = static_cast<size_t>(x) +
                                        static_cast<size_t>(y) * this->size +
                                        static_cast<size_t>(z) * this->size * this->size;

                                    if (grid[flat_idx].e == 0)
                                    {
                                        uint8_t final_r = r, final_g = g, final_b = b;

                                        // Mamy UV i trójkąt ma materiał z teksturą!
                                        if (has_uvs && mat_id >= 0 && mat_id < materials.size() && !materials[mat_id].diffuse_texname.empty())
                                        {
                                            std::string tex_name = materials[mat_id].diffuse_texname;

                                            if (texture_cache.count(tex_name) && texture_cache[tex_name].data)
                                            {
                                                TextureData& tex = texture_cache[tex_name];

                                                // INTERPOLACJA BARYCENTRYCZNA (Idealny punkt na zdjęciu)
                                                // Nasza funkcja rzutowania zwraca: u dla tri[0], v dla tri[1], w dla tri[2]
                                                glm::vec2 pixel_uv = uv[0] * u + uv[1] * v + uv[2] * w;

                                                // Zawijamy teksturę (jeśli UV wychodzi poza 0.0-1.0)
                                                float tex_u = pixel_uv.x - std::floor(pixel_uv.x);
                                                float tex_v = pixel_uv.y - std::floor(pixel_uv.y);

                                                // Pliki .obj często mają odwróconą oś pionową tekstur (V)
                                                tex_v = 1.0f - tex_v;

                                                // Przeliczamy UV na fizyczne piksele obrazka
                                                int px = std::clamp(static_cast<int>(tex_u * tex.width), 0, tex.width - 1);
                                                int py = std::clamp(static_cast<int>(tex_v * tex.height), 0, tex.height - 1);

                                                // Czytamy kolory RGB z pamięci
                                                int p_idx = (py * tex.width + px) * 3;
                                                final_r = tex.data[p_idx];
                                                final_g = tex.data[p_idx + 1];
                                                final_b = tex.data[p_idx + 2];
                                            }
                                        }

                                        // Zabezpieczenie przed "czarnym powietrzem" 
                                        grid[flat_idx].r = std::max((uint8_t)1, final_r);
                                        grid[flat_idx].g = std::max((uint8_t)1, final_g);
                                        grid[flat_idx].b = std::max((uint8_t)1, final_b);
                                        grid[flat_idx].e = 255;
                                        voxels_painted++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            index_offset += fv;
        }
    }

    std::cout << "Sukces! Zbudowano lita powloke z " << voxels_painted << " wokseli.\n";
    // Sprzątamy gigabajty zdjęć z RAM - u - nie są już potrzebne!
    for (auto& pair : texture_cache)
    {
        if (pair.second.data) stbi_image_free(pair.second.data);
    }
    return true;
}

std::optional<vrt::Dag::Voxel> vrt::VoxelModel::sample(glm::vec3 pos) const
{
    // Zamieniamy ze współrzędnych przestrzeni (gdzie (0,0,0) to środek) na indeksy tablicy
    int x = static_cast<int>(pos.x + (size * 0.5f));
    int y = static_cast<int>(pos.y + (size * 0.5f));
    int z = static_cast<int>(pos.z + (size * 0.5f));

    if (x >= 0 && x < size && y >= 0 && y < size && z >= 0 && z < size)
    {
        size_t flat_idx = static_cast<size_t>(x) +
            static_cast<size_t>(y) * size +
            static_cast<size_t>(z) * size * size;
        vrt::Dag::Voxel v = grid[flat_idx];

        // Zwracamy woksel TYLKO wtedy, gdy ma jakikolwiek zapisany kolor
        if (v.e != 0)
        { // Sprawdzamy tylko flagę, kolory r,g,b mogą być śmiało 0,0,0
            return v;
        }
        return std::nullopt;
    }
    return std::nullopt;
}