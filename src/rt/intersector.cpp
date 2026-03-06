#include <vrt/rt/intersector.hpp>
#include <glm/glm.hpp>

struct StackFrame
{
    std::uint32_t node_index;
    float tx, ty, tz;
    float t_exit;
    std::uint8_t oct_idx;
};

const vrt::Hit vrt::Intersector::intersect(const Ray& ray, std::uint32_t root_index, std::uint32_t max_depth, const glm::vec3& root_center) const noexcept
{
    constexpr float INF = std::numeric_limits<float>::infinity();

    // mirror ray
    glm::vec3 O = ray.origin;
    glm::vec3 invD = ray.direction_inverse;
    glm::vec3 C = root_center;
    std::uint8_t a = 0; // axis inversion mask

    if (invD.x < 0.0f) { O.x = -O.x; invD.x = -invD.x; C.x = -C.x; a |= 1; }
    if (invD.y < 0.0f) { O.y = -O.y; invD.y = -invD.y; C.y = -C.y; a |= 2; }
    if (invD.z < 0.0f) { O.z = -O.z; invD.z = -invD.z; C.z = -C.z; a |= 4; }

    float t_enter = 0.0f;
    float t_exit = INF;

    const float root_half = float(1u << max_depth);

    // Root slab test
    float t0x = (C.x - root_half - O.x) * invD.x;
    float t1x = (C.x + root_half - O.x) * invD.x;
    t_enter = std::max(t_enter, t0x);
    t_exit = std::min(t_exit, t1x);

    float t0y = (C.y - root_half - O.y) * invD.y;
    float t1y = (C.y + root_half - O.y) * invD.y;
    t_enter = std::max(t_enter, t0y);
    t_exit = std::min(t_exit, t1y);

    float t0z = (C.z - root_half - O.z) * invD.z;
    float t1z = (C.z + root_half - O.z) * invD.z;
    t_enter = std::max(t_enter, t0z);
    t_exit = std::min(t_exit, t1z);

    t_enter = std::max(t_enter, 0.0f);

    if (t_exit < t_enter)
        return { .t = INF, .normal = glm::vec3{0}, .voxel = Voxel::EMPTY };

    // Track the axis of the last ADVANCE step that set t_enter (0=X,1=Y,2=Z)
    // Initialize from root slab entry (argmax of t0x/t0y/t0z).
    std::uint8_t last_axis = 0;
    {
        float best = t0x;
        last_axis = 0;
        if (t0y > best) { best = t0y; last_axis = 1; }
        if (t0z > best) { /*best = t0z;*/ last_axis = 2; }
        // If we clamped t_enter to 0, last_axis is only a fallback; it will be overwritten on first ADVANCE.
    }

    StackFrame stack[32];
    int sp = 0;

    int depth = max_depth;
    std::uint32_t current_index = root_index;

    float half = root_half;

    // center axis (mid-planes of current node)
    float txm = (C.x - O.x) * invD.x;
    float tym = (C.y - O.y) * invD.y;
    float tzm = (C.z - O.z) * invD.z;

    std::uint8_t oct_idx =
        (std::uint8_t(txm <= t_enter) << 0) |
        (std::uint8_t(tym <= t_enter) << 1) |
        (std::uint8_t(tzm <= t_enter) << 2);

    float tx = (txm > t_enter) ? txm : INF;
    float ty = (tym > t_enter) ? tym : INF;
    float tz = (tzm > t_enter) ? tzm : INF;

    while (true)
    {
        // 1. PROCESS / DESCEND
        if (depth > 0)
        {
            const std::uint32_t child_index =
                blas_.Nodes()[current_index].indices[oct_idx ^ a];

            if (child_index != EMPTY)
            {
                // t_next = min(tx,ty,tz)
                float t_next = tx;
                if (ty < t_next) t_next = ty;
                if (tz < t_next) t_next = tz;

                const float child_t_exit = (t_exit < t_next) ? t_exit : t_next;

                // push frame
                stack[sp++] = { current_index, tx, ty, tz, t_exit, oct_idx };

                // descent
                half *= 0.5f;
                const float offset = half;

                txm += ((oct_idx & 1) ? +offset : -offset) * invD.x;
                tym += ((oct_idx & 2) ? +offset : -offset) * invD.y;
                tzm += ((oct_idx & 4) ? +offset : -offset) * invD.z;

                current_index = child_index;
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
        }
        else
        {
            // leaf
            Voxel v = blas_.Leaves()[current_index].voxels[oct_idx ^ a];
            if (v != Voxel::EMPTY)
            {
                // Face normal from last ADVANCE axis (cheap and correct for your stepping)
                glm::vec3 n{ 0.f, 0.f, 0.f };

                // In mirrored space, direction is always +, so we always enter through the "negative" face.
                if (last_axis == 0) n.x = -1.f;
                else if (last_axis == 1) n.y = -1.f;
                else                     n.z = -1.f;

                // un-mirror back to world space
                if (a & 1) n.x = -n.x;
                if (a & 2) n.y = -n.y;
                if (a & 4) n.z = -n.z;

                return { .t = t_enter, .normal = n, .voxel = v };
            }
        }

        // 3. ADVANCE / POP
        while (true)
        {
            int axis = 0;
            float t_next = tx;
            if (ty < t_next) { t_next = ty; axis = 1; }
            if (tz < t_next) { t_next = tz; axis = 2; }

            // POP
            if (t_next > t_exit)
            {
                if (sp == 0)
                    return { .t = INF, .normal = glm::vec3{0}, .voxel = Voxel::EMPTY };

                // ascend
                ++depth;

                const float offset = half;

                // pop frame
                const StackFrame& frame = stack[--sp];

                current_index = frame.node_index;
                tx = frame.tx;
                ty = frame.ty;
                tz = frame.tz;
                t_exit = frame.t_exit;
                oct_idx = frame.oct_idx;

                txm -= ((oct_idx & 1) ? +offset : -offset) * invD.x;
                tym -= ((oct_idx & 2) ? +offset : -offset) * invD.y;
                tzm -= ((oct_idx & 4) ? +offset : -offset) * invD.z;

                half *= 2.0f;

                continue;
            }

            // ADVANCE to next boundary
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