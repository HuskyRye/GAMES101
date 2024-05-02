//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Scene.hpp"
#include <fstream>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene, int ssaa)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            for (int b = 0; b < ssaa; ++b) {
                for (int a = 0; a < ssaa; ++a) {
                    // generate primary ray direction
                    float x = ((i + 1.0f / (2.0f * ssaa) + a * 1.0f / ssaa) * 2.0f / (float)scene.width - 1.0f) * imageAspectRatio * scale;
                    float y = (1.0f - (j + 1.0f / (2.0f * ssaa) + b * 1.0f / ssaa) * 2.0f / (float)scene.height) * scale;
                    // TODO: Find the x and y positions of the current pixel to get the direction
                    // vector that passes through it.
                    // Also, don't forget to multiply both of them with the variable *scale*, and
                    // x (horizontal) variable with the *imageAspectRatio*

                    Vector3f dir = normalize(Vector3f(x, y, -1.0f)); // Don't forget to normalize this direction!
                    Ray ray(eye_pos, dir);
                    framebuffer[m] += scene.castRay(ray, 0);
                }
            }
            framebuffer[m] = framebuffer[m] / (ssaa * ssaa);
            ++m;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
