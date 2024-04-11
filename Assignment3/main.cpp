#include <iostream>
#include <opencv2/opencv.hpp>

#include "OBJ_Loader.h"
#include "Shader.hpp"
#include "Texture.hpp"
#include "Triangle.hpp"
#include "global.hpp"
#include "rasterizer.hpp"

float degrees_to_radians(float degrees)
{
    return degrees * MY_PI / 180.0;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
        0, 2.5, 0, 0,
        0, 0, 2.5, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    zNear = -zNear;
    zFar = -zFar;

    // Perspective to Orthographic
    Eigen::Matrix4f persp_to_ortho;
    persp_to_ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // Orthographic, translate first, then scale
    float yTop = abs(zNear) * tan(degrees_to_radians(eye_fov / 2.0));
    float xRight = aspect_ratio * yTop;

    Eigen::Matrix4f translation;
    translation << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 1 / xRight, 0, 0, 0,
        0, 1 / yTop, 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    projection = scale * translation * persp_to_ortho * projection;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light {
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = { 0, 0, 0 };
    if (payload.texture) {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light { { 20, 20, 20 }, { 500, 500, 500 } };
    auto l2 = light { { -20, 20, 0 }, { 500, 500, 500 } };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity { 10, 10, 10 };
    Eigen::Vector3f eye_pos { 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f La = ka.array() * amb_light_intensity.array();

        float distance = (light.position - point).norm();
        Eigen::Vector3f I = light.intensity / distance / distance;

        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f Ld = kd.array() * I.array() * std::max(0.0f, normal.dot(l));

        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (v + l).normalized();
        Eigen::Vector3f Ls = ks.array() * I.array() * std::pow(std::max(0.0f, normal.dot(h)), p);

        result_color += La + Ld + Ls;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light { { 20, 20, 20 }, { 500, 500, 500 } };
    auto l2 = light { { -20, 20, 0 }, { 500, 500, 500 } };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity { 10, 10, 10 };
    Eigen::Vector3f eye_pos { 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = { 0, 0, 0 };
    for (auto& light : lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f La = ka.array() * amb_light_intensity.array();

        float distance = (light.position - point).norm();
        Eigen::Vector3f I = light.intensity / distance / distance;

        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f Ld = kd.array() * I.array() * std::max(0.0f, normal.dot(l));

        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (v + l).normalized();
        Eigen::Vector3f Ls = ks.array() * I.array() * std::pow(std::max(0.0f, normal.dot(h)), p);

        result_color += La + Ld + Ls;
    }

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light { { 20, 20, 20 }, { 500, 500, 500 } };
    auto l2 = light { { -20, 20, 0 }, { 500, 500, 500 } };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity { 10, 10, 10 };
    Eigen::Vector3f eye_pos { 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t { x * y / sqrt(x * x + z * z),
        sqrt(x * x + z * z),
        z * y / sqrt(x * x + z * z) };
    // Vector b = n cross product t
    Eigen::Vector3f b = normal.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t, b, normal;
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];
    int w = payload.texture->width;
    int h = payload.texture->height;
    float hCurr = payload.texture->getColor(u, v).norm();
    float hRight = payload.texture->getColor(u + 1.0f / w, v).norm();
    float hUp = payload.texture->getColor(u, v + 1.0f / h).norm();
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float dU = kh * kn * (hRight - hCurr);
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kh * kn * (hUp - hCurr);
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln { -dU, -dV, 1.0f };
    // Position p = p + kn * n * h(u,v)
    point += kn * normal * hCurr;
    // Normal n = normalize(TBN * ln)
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f La = ka.array() * amb_light_intensity.array();

        float distance = (light.position - point).norm();
        Eigen::Vector3f I = light.intensity / distance / distance;

        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f Ld = kd.array() * I.array() * std::max(0.0f, normal.dot(l));

        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (v + l).normalized();
        Eigen::Vector3f Ls = ks.array() * I.array() * std::pow(std::max(0.0f, normal.dot(h)), p);

        result_color += La + Ld + Ls;
    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light { { 20, 20, 20 }, { 500, 500, 500 } };
    auto l2 = light { { -20, 20, 0 }, { 500, 500, 500 } };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity { 10, 10, 10 };
    Eigen::Vector3f eye_pos { 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t { x * y / sqrt(x * x + z * z),
        sqrt(x * x + z * z),
        z * y / sqrt(x * x + z * z) };
    // Vector b = n cross product t
    Eigen::Vector3f b = normal.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t, b, normal;
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];
    int w = payload.texture->width;
    int h = payload.texture->height;
    float hCurr = payload.texture->getColor(u, v).norm();
    float hRight = payload.texture->getColor(u + 1.0f / w, v).norm();
    float hUp = payload.texture->getColor(u, v + 1.0f / h).norm();
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float dU = kh * kn * (hRight - hCurr);
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kh * kn * (hUp - hCurr);
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln { -dU, -dV, 1.0f };
    // Normal n = normalize(TBN * ln)
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";
    // std::string obj_path = "../models/cube/";
    // std::string obj_path = "../models/rock/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    // bool loadout = Loader.LoadFile("../models/cube/cube.obj");
    // bool loadout = Loader.LoadFile("../models/rock/rock.obj");
    for (auto mesh : Loader.LoadedMeshes) {
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            Triangle* t = new Triangle();
            for (int j = 0; j < 3; j++) {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700, 1);

    auto texture_path = "spot_texture.png";
    // auto texture_path = "hmap.jpg";
    // auto texture_path = "wall1.tif";
    // auto texture_path = "rock.png";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    if (argc >= 2) {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture") {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "normal") {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "phong") {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "bump") {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "displacement") {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = { 0, 0, 10 };

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    int key = 0;
    int frame_count = 0;
    auto start = cv::getTickCount();

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        // r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        ++frame_count;
        if (frame_count == 100) {
            double time = (cv::getTickCount() - start) / cv::getTickFrequency();
            double fps = frame_count / time;
            std::cout << "fps: " << fps << '\n';
            frame_count = 0;
            start = cv::getTickCount();
        }

        cv::imshow("image", image);
        // cv::imwrite(filename, image);
        key = cv::waitKey(1);

        if (key == 'a') {
            angle -= 10;
        } else if (key == 'd') {
            angle += 10;
        }
    }
    return 0;
}
