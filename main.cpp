#include "GUI/GUI.hpp"
#include "SDL2/SDL.h"
#include "SDL2/SDL_timer.h"
#include "SDL_render.h"
#include "SDL_surface.h"

#include <atomic>
#include <new>

int main()
{
    std::atomic_bool continue_running {true};
    ELN::GUI gui {continue_running};
    gui.init();
    constexpr uint32_t RENDER_FLAGS = SDL_RENDERER_ACCELERATED;
    SDL_Renderer *renderer = SDL_CreateRenderer(gui.window(), -1, RENDER_FLAGS);
    constexpr size_t WIDTH {1024};
    constexpr size_t HEIGHT {1024};
    constexpr unsigned CHANNELS {4};

    std::array<uint8_t, WIDTH * HEIGHT * CHANNELS> image {};
    SDL_Surface *surface =
        SDL_CreateRGBSurfaceFrom(image.data(), WIDTH, HEIGHT, CHANNELS * 8, CHANNELS * WIDTH,
                                 0xff000000, 0x00ff0000, 0x0000ff00, 0x00000000);
    if (not surface)
        throw std::bad_alloc {};

    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);

    gui.runEveryFrame(
        [&]()
        {
            ImGui::Begin("SDL_Renderer Texture Test");
            ImGui::Text("pointer = %p", (void *)texture);
            ImGui::Text("size = %d x %d", WIDTH, HEIGHT);
            ImGui::Image((ImTextureID)(intptr_t)texture, ImVec2((float)WIDTH, (float)HEIGHT));
            ImGui::End();
        });

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
}
