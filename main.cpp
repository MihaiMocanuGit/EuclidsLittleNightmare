#include "GUI/GUI.hpp"

#include <atomic>
#include <cmath>

std::array<float, 3> vectorField(std::array<float, 3> x)
{
    std::array<float, 3> v = {std::sin(x[0] + x[1]), 0, 0};
    float norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
    return v;
}

int main()
{
    std::atomic_bool continue_running {true};
    ELN::GUI gui {continue_running};
    gui.init();

    constexpr unsigned WIDTH {1024};
    constexpr unsigned HEIGHT {1024};
    constexpr unsigned CHANNELS {3};

    std::array<uint8_t, WIDTH * HEIGHT * CHANNELS> image {};
    constexpr float cameraOrigin[3] = {0.0f, 0.0f, 0.0f};
    constexpr float distanceToViewport {100.0f};
    constexpr float stepsize = 0.5f;

    constexpr float sphereCenter[3] = {0.0f, 0.0f, 200.0f};
    constexpr float sphereRadius = 75.0f;

    for (unsigned pixel_y {0}; pixel_y < HEIGHT; ++pixel_y)
    {
        for (unsigned pixel_x {0}; pixel_x < WIDTH; ++pixel_x)
        {
            float xViewport = static_cast<float>(pixel_x) - WIDTH / 2.0f;
            float yViewport = static_cast<float>(pixel_y) - HEIGHT / 2.0f;
            float zViewport = distanceToViewport;

            float rayDirVector[3] = {xViewport - cameraOrigin[0], yViewport - cameraOrigin[1],
                                     zViewport - cameraOrigin[2]};
            float rayDirVectorNorm =
                std::sqrt(rayDirVector[0] * rayDirVector[0] + rayDirVector[1] * rayDirVector[1] +
                          rayDirVector[2] * rayDirVector[2]);

            rayDirVector[0] /= rayDirVectorNorm;
            rayDirVector[1] /= rayDirVectorNorm;
            rayDirVector[2] /= rayDirVectorNorm;

            float x_0 = xViewport;
            float y_0 = yViewport;
            float z_0 = zViewport;

            for (size_t steps {0}; steps < 5000; ++steps)
            {
                const auto field = vectorField({x_0, y_0, z_0});
                float x = (rayDirVector[0] + 0.1f * field[0]) * stepsize + x_0;
                float y = (rayDirVector[1] + 0.1f * field[1]) * stepsize + y_0;
                float z = (rayDirVector[2] + 0.1f * field[2]) * stepsize + z_0;

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

                float xs_2 = x - sphereCenter[0];
                xs_2 *= xs_2;

                float ys_2 = y - sphereCenter[1];
                ys_2 *= ys_2;

                float zs_2 = z - sphereCenter[2];
                zs_2 *= zs_2;

                if (xs_2 + ys_2 + zs_2 <= sphereRadius * sphereRadius)
                {
                    image[pixel_y * 3 * WIDTH + 3 * pixel_x + 0] = 255;
                    image[pixel_y * 3 * WIDTH + 3 * pixel_x + 1] = 255;
                    image[pixel_y * 3 * WIDTH + 3 * pixel_x + 2] = 255;
                    break;
                }
                else
                {
                    image[pixel_y * 3 * WIDTH + 3 * pixel_x + 0] = 0;
                    image[pixel_y * 3 * WIDTH + 3 * pixel_x + 1] = 0;
                    image[pixel_y * 3 * WIDTH + 3 * pixel_x + 2] = 0;
                }
            }
        }
    }

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
            ImGui::Begin("OpenGL Texture Text");
            ImGui::Text("pointer = %x", image_texture);
            ImGui::Text("size = %d x %d", WIDTH, HEIGHT);
            ImGui::Image((ImTextureID)(intptr_t)image_texture, ImVec2(WIDTH, HEIGHT));
            ImGui::End();
        });
}
