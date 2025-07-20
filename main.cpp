#include "Eigen/Geometry"
#include "GUI/GUI.hpp"
#include "Scene/TreeBVH/TreeBVH.hpp"
#include "Utils/utils.hpp"
#include "imgui.h"

#include <atomic>
#include <cmath>
#include <execution>
#include <iostream>
#include <numeric>
#include <random>

// CAUTION - ENTER AT YOUR OWN RISK!
//
// The current state of main.cpp file represents a sandbox like environment for experimenting with
// different concepts. This is not the "final application".
//
// No recommended coding practices are followed in this wild west.

std::array<float, 3> vectorField(std::array<float, 3> x)
{
    std::array<float, 3> v = {400 - x[0], 400 - x[1], 400 - x[2]};
    float norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (norm >= 0.01f)
    {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
    return v;
}

struct Elipsoid_t
{
    Eigen::Vector3f center;
    Eigen::Vector3f coeff;
    std::array<uint8_t, 3> color;

    constexpr bool checkHit(const Eigen::Vector3f &position) const noexcept
    {
        float coeffSum {0};
        for (unsigned dim {0}; dim < 3; ++dim)
            coeffSum += std::pow(position[dim] - center[dim], 2.0f) / std::pow(coeff[dim], 2.0f);
        return coeffSum <= 1.0f;
    }

    ELN::BVH::Boundary_t<float> boundary() const noexcept
    {
        Eigen::Vector3f min {center - coeff};
        Eigen::Vector3f max {center + coeff};
        return {min, max};
    }
};

int main()
{

    constexpr unsigned WIDTH {1600};
    constexpr unsigned HEIGHT {900};
    constexpr unsigned CHANNELS {3};
    const ELN::BVH::Boundary_t<float> WORLD_BD {
        1.0f * Eigen::Vector3f {-1.0f * WIDTH, -1.0f * HEIGHT, 0.0f},
        1.0f * Eigen::Vector3f {WIDTH, HEIGHT, HEIGHT}};
    constexpr size_t NO_ELEMENTS = 500;
    std::vector<ELN::BVH::CompactTree<Elipsoid_t>::ElemBdrPair> elements;
    elements.reserve(NO_ELEMENTS);
    std::mt19937 gen;
    for (size_t i {0}; i < NO_ELEMENTS; ++i)
    {
        const ELN::BVH::Boundary_t<float> OBJ_BD {Eigen::Vector3f {19.0f, 19.0f, 19.0f},
                                                  Eigen::Vector3f {20.0f, 20.0f, 20.0f}};
        const auto color = [&]()
        {
            std::uniform_int_distribution<uint8_t> distrib(64, 255);
            return std::array<uint8_t, 3> {distrib(gen), distrib(gen), distrib(gen)};
        };
        Elipsoid_t object {.center = WORLD_BD.sample(), .coeff = OBJ_BD.sample(), .color = color()};
        elements.emplace_back(object, object.boundary());
    }
    const ELN::BVH::CompactTree<Elipsoid_t> tree {std::move(elements)};

    std::atomic_bool continue_running {true};
    ELN::GUI gui {continue_running};
    gui.init();

    std::array<uint8_t, WIDTH * HEIGHT * CHANNELS> image {};
    constexpr float distanceToViewport {HEIGHT / 2.0f};
    constexpr float cameraOrigin[3] = {0.0f, 0.0f, -distanceToViewport};
    constexpr float fastStepSize = 1.0f;
    constexpr float preciseStepSize = 1.0f;

    constexpr auto yPixels = ELN::Utils::array_iota<HEIGHT>(0U);
    constexpr auto xPixels = ELN::Utils::array_iota<HEIGHT>(0U);

    std::thread worker {
        [&]()
        {
            std::for_each(
                std::execution::par_unseq, yPixels.begin(), yPixels.end(),
                [&](unsigned pixel_y)
                {
                    for (unsigned pixel_x {0}; pixel_x < WIDTH; ++pixel_x)
                    {
                        float xViewport = static_cast<float>(pixel_x) - WIDTH / 2.0f;
                        float yViewport = static_cast<float>(pixel_y) - HEIGHT / 2.0f;
                        float zViewport = distanceToViewport + cameraOrigin[2];

                        float rayDirVector[3] = {xViewport - cameraOrigin[0],
                                                 yViewport - cameraOrigin[1],
                                                 zViewport - cameraOrigin[2]};
                        float rayDirVectorNorm = std::sqrt(rayDirVector[0] * rayDirVector[0] +
                                                           rayDirVector[1] * rayDirVector[1] +
                                                           rayDirVector[2] * rayDirVector[2]);

                        rayDirVector[0] /= rayDirVectorNorm;
                        rayDirVector[1] /= rayDirVectorNorm;
                        rayDirVector[2] /= rayDirVectorNorm;

                        float x_0 = xViewport;
                        float y_0 = yViewport;
                        float z_0 = zViewport;

                        float stepsize = fastStepSize;
                        const float MAX_LEN {std::sqrtf(std::powf(WORLD_BD.max()[0], 2.0f) +
                                                        std::powf(WORLD_BD.max()[1], 2.0f) +
                                                        std::powf(WORLD_BD.max()[2], 2.0f)) *
                                             1.2f};
                        for (float arcLen {0}; arcLen < MAX_LEN; arcLen += stepsize)
                        {
                            const auto field = vectorField({x_0, y_0, z_0});
                            float x = (rayDirVector[0] + 0.005f * field[0]) * stepsize + x_0;
                            float y = (rayDirVector[1] + 0.005f * field[1]) * stepsize + y_0;
                            float z = (rayDirVector[2] + 0.005f * field[2]) * stepsize + z_0;

                            const auto setColorPixel = [&](const std::array<uint8_t, 3> &color)
                            {
                                image[pixel_y * 3 * WIDTH + 3 * pixel_x + 0] = color[0];
                                image[pixel_y * 3 * WIDTH + 3 * pixel_x + 1] = color[1];
                                image[pixel_y * 3 * WIDTH + 3 * pixel_x + 2] = color[2];
                            };
                            if (WORLD_BD.exteriorDistance(Eigen::Vector3f {x, y, z}) > 50.0f)
                            {
                                setColorPixel({0, 0, 0});
                                break;
                            }
                            rayDirVector[0] = x - x_0;
                            rayDirVector[1] = y - y_0;
                            rayDirVector[2] = z - z_0;
                            rayDirVectorNorm = std::sqrt(rayDirVector[0] * rayDirVector[0] +
                                                         rayDirVector[1] * rayDirVector[1] +
                                                         rayDirVector[2] * rayDirVector[2]);
                            rayDirVector[0] /= rayDirVectorNorm;
                            rayDirVector[1] /= rayDirVectorNorm;
                            rayDirVector[2] /= rayDirVectorNorm;

                            x_0 = x;
                            y_0 = y;
                            z_0 = z;

                            const auto hitQuery = tree.queryPosition({x, y, z});
                            bool hitObject {false};
                            setColorPixel({0, 0, 0});
                            for (const auto &boundary : hitQuery)
                            {
                                stepsize = preciseStepSize;
                                if (boundary.get().checkHit({x, y, z}))
                                {
                                    setColorPixel(boundary.get().color);
                                    hitObject = true;
                                    break;
                                }
                            }
                            if (hitObject)
                                break;
                            else if (hitQuery.empty())
                                stepsize = fastStepSize;
                        }
                    }
                });
        }};
    worker.detach();

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 image.data());

    gui.runEveryFrame(
        [&]()
        {
#ifdef NDEBUG
#else
            bool demo = true;
            ImGui::ShowDemoWindow(&demo);
#endif
            // Upload current pixels into texture written by separate thread.
            // This most probably is undefined behaviour as the texture is updated in another thread
            // without any synchronisation/atomic structure.
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE,
                         image.data());

            ImGui::Begin("Result");
            ImVec2 windowSize {ImGui::GetWindowSize()};
            float aspectRatio {static_cast<float>(WIDTH) / HEIGHT};
            ImGui::Image((ImTextureID)(intptr_t)image_texture,
                         ImVec2(windowSize.y * aspectRatio, windowSize.y));
            ImGui::End();
        });
}
