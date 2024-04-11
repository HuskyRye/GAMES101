#include "Texture.hpp"
//
// Created by LEI XU on 4/27/19.
//

Eigen::Vector3f Texture::getColorBilinear(float u, float v)
{
    u = std::clamp(u, 0.0f, 1.0f);
    v = std::clamp(v, 0.0f, 1.0f);

    float u_img = u * width;
    float v_img = (1 - v) * height;

    int u0 = std::floor(u_img - 0.5);
    int u1 = u0 + 1;
    int v0 = std::floor(v_img - 0.5);
    int v1 = v0 + 1;

    u0 = std::max(u0, 0);
    u1 = std::min(u1, width - 1);
    v0 = std::max(v0, 0);
    v1 = std::min(v1, height - 1);

    // uchar could result to overflow!
    auto convert = [](const cv::Vec3b& vec) -> Eigen::Vector3f {
        return Eigen::Vector3f(static_cast<float>(vec[0]), static_cast<float>(vec[1]), static_cast<float>(vec[2]));
    };

    auto color00 = convert(image_data.at<cv::Vec3b>(v0, u0));
    auto color01 = convert(image_data.at<cv::Vec3b>(v0, u1));
    auto color10 = convert(image_data.at<cv::Vec3b>(v1, u0));
    auto color11 = convert(image_data.at<cv::Vec3b>(v1, u1));

    float s = u_img - (u0 + 0.5);
    float t = v_img - (v0 + 0.5);
    auto color0 = color00 + s * (color01 - color00);
    auto color1 = color10 + s * (color11 - color10);
    auto color = color0 + t * (color1 - color0);

    return color;
}
