//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Scene.hpp"
#include <fstream>
#include <mutex>
#include <thread>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.0001f;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 1024;
    std::cout << "SPP: " << spp << "\n";

    int num_threads = std::thread::hardware_concurrency();
    std::cout << "num_threads: " << num_threads << "\n";
    std::vector<std::thread> threads;
    int rows_per_thread = scene.height / num_threads;
    int completed_rows = 0;
    std::mutex mtx;

    for (int t = 0; t < num_threads; ++t) {
        int start_row = t * rows_per_thread;
        int end_row = (t + 1 == num_threads) ? scene.height : start_row + rows_per_thread;
        threads.emplace_back([=, &scene, &framebuffer, &completed_rows, &mtx] {
            for (uint32_t j = start_row; j < end_row; ++j) {
                for (uint32_t i = 0; i < scene.width; ++i) {
                    for (int k = 0; k < spp; k++) {
                        int index = j * scene.width + i;
                        // generate primary ray direction
                        float x = (2 * (i + get_random_float()) / (float)scene.width - 1) * imageAspectRatio * scale;
                        float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;
                        Vector3f dir = normalize(Vector3f(-x, y, 1));
                        framebuffer[index] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                    }
                }
                mtx.lock();
                ++completed_rows;
                UpdateProgress(completed_rows / (float)scene.height);
                mtx.unlock();
            }
        });
    }
    UpdateProgress(1.f);

    for (auto& thread : threads)
        thread.join();

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
