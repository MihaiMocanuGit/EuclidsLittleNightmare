#include "Eigen/Geometry"
///////
#include "glad/glad.h"
///////
#include "GUI/GUI.hpp"
#include "Scene/TreeBVH/TreeBVH.hpp"
#include "Utils/utils.hpp"
#include "imgui.h"

#include <atomic>
#include <cmath>
#include <execution>
#include <numbers>
#include <random>

// CAUTION - ENTER AT YOUR OWN RISK!
//
// The current state of main.cpp file represents a sandbox like environment used for experimenting
// with different concepts. This is not the "final application".
//
// No recommended coding practices are followed in this godless place.
//
// Rest at this bonfire to replenish your strength before your adventure
//
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣤⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣿⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣯⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣿⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⢺⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡏⣧⣀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢶⣿⣋⣟⠭⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣿⣭⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡏⢮⣳⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⡿⣦⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⠢⣽⣅⠀⠀⠀⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡆⢤⣿⡇⠀⠀⠀⠀⣸⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢷⠸⣞⡇⠀⠀⠀⠀⡏⢧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡄⣿⣷⠀⠀⠀⠀⢻⡈⢣⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣇⢸⣿⡆⠀⠀⠀⠀⢳⣬⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡈⣿⣧⠀⠀⢠⡄⣸⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⡇⢹⣿⡀⠀⢸⢧⠟⢹⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠸⣿⡇⣠⠋⢾⣾⢸⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡖⠀⠸⣿⣶⣿⣷⡏⢰⡿⢿⠏⣸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡇⠀⣴⢋⣿⣿⣿⠇⡟⠁⣏⠀⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡞⣏⢦⠇⢸⡿⢿⠋⢀⣤⣀⡘⢦⡟⢸⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⣿⠃⢀⣴⡆⠀⠀⠈⣹⣿⡷⠆⠀⣧⠈⢿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠜⢁⡴⠋⡀⠙⢄⠀⣰⣿⣟⠓⠀⠀⢉⣴⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⢰⡏⡠⠊⢀⡴⣇⠀⢀⡞⠉⠛⠀⡀⢀⣄⣩⠌⠙⢦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⢣⠶⠖⠊⢀⣈⠉⣹⡷⢀⣴⡯⠔⣛⡵⠁⣠⡏⠸⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⢹⡿⢿⠟⠀⣰⡞⠉⣿⡷⠇⠃⣠⢴⣶⣾⡋⢀⡴⣽⠁⠀⠘⣏⣀⢰⣆⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⣠⣶⣶⣅⣠⣶⠀⠒⠟⢁⡴⠋⠀⠀⠀⢹⣿⣿⡋⣧⢸⡇⡏⣀⣀⠀⠙⣿⣉⠙⢤⡄⠀⠀⠀
// ⠀⠀⣠⣴⣺⢿⣿⣿⡛⠛⠿⠿⣯⣷⡲⣶⣟⣻⡀⠀⣠⣿⣿⣖⣸⣨⣿⠿⠛⣻⣿⣶⣾⣾⠇⠀⠻⣄⠀⠀
// ⠀⣾⢟⠿⠿⢶⣮⡙⢏⢢⡀⢠⡌⣿⣿⡿⠟⡿⢳⣼⣿⣿⣿⣾⣿⣧⣤⣤⣤⣿⣿⣭⣿⠁⠀⠀⣀⣈⣧⠀
// ⢺⣥⢿⠾⠿⠿⠿⡿⠚⢋⣠⠯⣿⢉⢉⠻⠾⠛⢿⣿⠻⠿⢛⢋⣤⣯⣭⠽⠶⣾⣻⢿⣻⢿⠶⢛⣻⡿⢽⠄
//
// Don't you dare go Hollow!

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

struct Ellipsoid_t
{
    Eigen::Vector3f center;
    Eigen::Vector3f coeff;
    std::array<uint8_t, 3> color;

    constexpr bool checkHit(const Eigen::Vector3f &position) const noexcept
    {
        float coeffSum {0};
        for (unsigned dim {0u}; dim < 3u; ++dim)
            coeffSum += std::pow(position[dim] - center[dim], 2.0f) / std::pow(coeff[dim], 2.0f);
        return coeffSum <= 1.0f;
    }

    float hitAngle(const Eigen::Vector3f &position, const Eigen::Vector3f &direction) const noexcept

    {
        // Ellipsoid: (x-xc)^2/a^2 + (y-yc)^2/b^2 + (y-yc)^2/c^2 = 1
        //
        // Line: [x,y,z] = direction * t + position
        //
        // Intersection:
        // (xd*t + xp - xc)^2/a^2 + (yd*t + yp - yc)^2/b^2 + (zd*t + zp - zc)^2/c^2 = 1
        //
        // Equation in t:
        // t^2 * (xd^2/a^2 + yd^2/b^2 + zd^2/c^2)
        // + t * (xd*(xp-xc)/a^2 + yd*(yp-yc)/b^2 + zd*(zp-zc)/c^2)
        // + (xp-xc)^2/a^2 + (yp-yc)^2/b^2 + (zp-zc)^2/c^2 - 1 = 0
        //
        // Rewrite as: t^2 * A + t * B + C = 0
        //
        // Discriminant: delta = B^2 - 4*A*C
        //
        // If delta < 0 then the ray missed.
        // Else if delta >= 0 then there exists at least one intersection point. Find the smallest
        // root of t.
        //
        // Compute the gradient on the ellipsoid surface at the first intersection. This will yield
        // a normal vector.
        //
        // Compute the cosine of the angle between the direction vector and normal vector. It can be
        // easily computed from their dot product.

        float A = 0;
        for (unsigned dim {0u}; dim < 3u; ++dim)
            A += std::powf(direction[dim], 2.0f) / std::powf(coeff[dim], 2.0f);

        float B = 0;
        for (unsigned dim {0u}; dim < 3u; ++dim)
            B += std::powf(direction[dim] * (position[dim] - center[dim]), 2.0f) /
                 std::powf(coeff[dim], 2.0f);

        float C = 0;
        for (unsigned dim {0u}; dim < 3u; ++dim)
            C += std::powf(position[dim] - center[dim], 2.0f) / std::powf(coeff[dim], 2.0f);

        float delta = std::powf(B, 2.0f) - 4 * A * C;
        if (delta >= 0)
        {
            // It can be observed that t1 < t2 and we only care about the first one: t1
            float t1 {(-B - std::sqrt(delta)) / (2 * A)};
            Eigen::Vector3f intersect {direction * t1 + position};
            Eigen::Vector3f gradient {
                2 * intersect[0] / std::powf(coeff[0], 2.0f),
                2 * intersect[1] / std::powf(coeff[1], 2.0f),
                2 * intersect[2] / std::powf(coeff[2], 2.0f),
            };
            gradient.normalize();
            Eigen::Vector3f dirVector {direction.normalized()};
            float cosine {gradient.dot(dirVector) / (gradient.norm() * dirVector.norm())};
            return cosine;
        }

        return 0.0f;
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
    constexpr size_t NO_ELEMENTS = 1000;
    std::vector<ELN::BVH::CompactTree<Ellipsoid_t>::ElemBdrPair> elements;
    elements.reserve(NO_ELEMENTS);
    std::mt19937 gen;
    for (size_t i {0}; i < NO_ELEMENTS; ++i)
    {
        const ELN::BVH::Boundary_t<float> OBJ_BD {Eigen::Vector3f {19.0f, 19.0f, 19.0f},
                                                  Eigen::Vector3f {20.0f, 20.0f, 20.0f}};
        const auto color = [&]()
        {
            std::uniform_int_distribution<uint8_t> distrib(64, 255);
            Eigen::Vector3f colorVec(distrib(gen), distrib(gen), distrib(gen));
            colorVec.normalize();
            colorVec *= 255;

            std::array<uint8_t, 3> byteColor;
            for (unsigned dim = 0; dim < 3; ++dim)
                byteColor[dim] = std::clamp(static_cast<int>(colorVec[dim]), 0, 255);

            return byteColor;
        };
        Ellipsoid_t object {
            .center = WORLD_BD.sample(), .coeff = OBJ_BD.sample(), .color = color()};
        elements.emplace_back(object, object.boundary());
    }
    const ELN::BVH::CompactTree<Ellipsoid_t> tree {std::move(elements)};

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
                                    auto pixelColor = boundary.get().color;
                                    float cosOfAngle {boundary.get().hitAngle(
                                        {x, y, z},
                                        {rayDirVector[0], rayDirVector[1], rayDirVector[2]})};
                                    pixelColor[0] *= cosOfAngle;
                                    pixelColor[1] *= cosOfAngle;
                                    pixelColor[2] *= cosOfAngle;

                                    setColorPixel(pixelColor);
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
