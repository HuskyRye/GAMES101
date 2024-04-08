//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vector>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// static bool insideTriangle(int x, int y, const Vector3f* _v)
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p { x, y, 1.0f };

    Vector3f v0v1 = _v[1] - _v[0];
    Vector3f v1v2 = _v[2] - _v[1];
    Vector3f v2v0 = _v[0] - _v[2];

    Vector3f v0p = p - _v[0];
    Vector3f v1p = p - _v[1];
    Vector3f v2p = p - _v[2];

    float z1 = v0v1.cross(v0p).z();
    float z2 = v1v2.cross(v1p).z();
    float z3 = v2v0.cross(v2p).z();

    return (z1 >= 0 && z2 >= 0 && z3 >= 0)
        || (z1 <= 0 && z2 <= 0 && z3 <= 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1, c2, c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };
        // Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto& vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = -(vert.z() * f1 - f2); // reverse z to make sure they are all positive
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

std::vector<Eigen::Vector3f>& rst::rasterizer::frame_buffer()
{
    int sample_count = ssaa * ssaa;
    for (int i = 0; i < width * height; ++i) {
        for (int j = 0; j < sample_count; ++j)
            frame_buf[i] += ssaa_buf[i * sample_count + j];
        frame_buf[i] /= sample_count;
    }
    return frame_buf;
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    int x_min = floor(std::min({ v[0].x(), v[1].x(), v[2].x() }));
    int x_max = ceil(std::max({ v[0].x(), v[1].x(), v[2].x() }));
    int y_min = floor(std::min({ v[0].y(), v[1].y(), v[2].y() }));
    int y_max = ceil(std::max({ v[0].y(), v[1].y(), v[2].y() }));

    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int pixel_y = y_min; pixel_y < y_max; ++pixel_y) {
        for (int pixel_x = x_min; pixel_x < x_max; ++pixel_x) {
            // for every ssaa pixel
            for (int j = 0; j < ssaa; ++j) {
                for (int i = 0; i < ssaa; ++i) {
                    float sample_x = pixel_x + 1.0f / (2 * ssaa) + i * 1.0f / ssaa;
                    float sample_y = pixel_y + 1.0f / (2 * ssaa) + j * 1.0f / ssaa;
                    if (insideTriangle(sample_x, sample_y, t.v)) {
                        // If so, use the following code to get the interpolated z value.
                        auto [alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        int index = get_index(pixel_x, pixel_y) * ssaa * ssaa + (j * ssaa + i);
                        if (z_interpolated < depth_buf[index]) {
                            ssaa_buf[index] = t.getColor();
                            depth_buf[index] = z_interpolated;
                        }
                    }
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f { 0, 0, 0 });
        std::fill(ssaa_buf.begin(), ssaa_buf.end(), Eigen::Vector3f { 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h, int ssaa)
    : width(w)
    , height(h)
    , ssaa(ssaa)
{
    frame_buf.resize(w * h);
    ssaa_buf.resize(w * h * ssaa * ssaa);
    depth_buf.resize(w * h * ssaa * ssaa);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    // old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
