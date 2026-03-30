#pragma once

#include <algorithm>
#include <cstddef>
#include <glm/glm.hpp>

inline glm::vec3 SampleThermalGradient(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    struct Stop
    {
        float pos;
        glm::vec3 color;
    };
    static const Stop stops[] = {
        {0.0f, glm::vec3(0.03f, 0.07f, 0.28f)},
        {0.25f, glm::vec3(0.00f, 0.45f, 0.95f)},
        {0.50f, glm::vec3(0.00f, 0.90f, 0.45f)},
        {0.75f, glm::vec3(0.98f, 0.68f, 0.12f)},
        {1.0f, glm::vec3(1.00f, 0.20f, 0.05f)},
    };

    for (std::size_t i = 1; i < sizeof(stops) / sizeof(stops[0]); ++i)
    {
        if (t <= stops[i].pos)
        {
            float start = stops[i - 1].pos;
            float end = stops[i].pos;
            float span = std::max(1e-5f, end - start);
            float local = (t - start) / span;
            return glm::mix(stops[i - 1].color, stops[i].color, local);
        }
    }

    return stops[sizeof(stops) / sizeof(stops[0]) - 1].color;
}
