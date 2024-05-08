//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"

enum MaterialType { DIFFUSE,
    MICROFACET };

class Material {
private:
    // Compute reflection direction
    Vector3f reflect(const Vector3f& I, const Vector3f& N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) {
            cosi = -cosi;
        } else {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {
            std::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        } else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f& a, const Vector3f& N)
    {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)) {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
        } else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    // normal distribution function
    // Trowbridge-Reitz 1975 (GGX 2007)
    float D_GGX(float alpha, float NoH)
    {
        float alphaSq = alpha * alpha;
        float denom = (alphaSq - 1.0f) * NoH * NoH + 1.0f;
        denom = std::max(EPSILON, denom);
        return alphaSq / (M_PI * denom * denom); // Numerical accuracy issues
    }

public:
    MaterialType m_type;
    // Vector3f m_color;
    Vector3f m_emission;
    float ior;
    float roughness;
    Vector3f Kd, Ks;
    float specularExponent;
    // Texture tex;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    // inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
};

Material::Material(MaterialType t, Vector3f e)
{
    m_type = t;
    // m_color = c;
    m_emission = e;
}

MaterialType Material::getType() { return m_type; }
/// Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission()
{
    if (m_emission.norm() > EPSILON)
        return true;
    else
        return false;
}

Vector3f Material::getColorAt(double u, double v)
{
    return Vector3f();
}

Vector3f Material::sample(const Vector3f& wi, const Vector3f& N)
{
    switch (m_type) {
    case DIFFUSE: {
        // uniform sample on the hemisphere
        float x_1 = get_random_float(), x_2 = get_random_float();
        float z = std::fabs(1.0f - 2.0f * x_1);
        float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
        Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
        return toWorld(localRay, N);
        break;
    }
    case MICROFACET: {
        float alpha = roughness * roughness;
        float alphaSq = alpha * alpha;
        float e_1 = get_random_float();
        float e_2 = get_random_float();
        float phi = 2 * M_PI * e_1;
        float theta = std::acos(sqrt((1 - e_2) / (e_2 * (alphaSq - 1.0f) + 1.0f)));
        Vector3f localRay(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
        Vector3f h = toWorld(localRay, N);
        return reflect(-wi, h);
        break;
    }
    }
}

float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
    if (dotProduct(wo, N) < EPSILON || dotProduct(wi, N) < EPSILON)
        return 0.0f;
    Vector3f h = normalize(wo + wi);
    switch (m_type) {
    case DIFFUSE: {
        // uniform sample probability 1 / (2 * PI)
        return 0.5f / M_PI;
        break;
    }
    case MICROFACET: {
        float alpha = roughness * roughness;
        float alphaSq = alpha * alpha;
        float NoH = dotProduct(N, h);

        float D = D_GGX(alpha, NoH);
        return D * NoH / (4.0f * dotProduct(wo, h));
    }
    }
}

Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
    float NoV = dotProduct(N, wo);
    float NoL = dotProduct(N, wi);
    if (NoV < EPSILON || NoL < EPSILON)
        return Vector3f(0.0f);

    switch (m_type) {
    case DIFFUSE: {
        // calculate the contribution of diffuse model
        Vector3f diffuse = Kd / M_PI;
        return diffuse;
        break;
    }
    case MICROFACET: {
        float alpha = roughness * roughness;
        float alphaSq = alpha * alpha;

        Vector3f h = normalize(wi + wo);

        float LoH = clamp(0.0f, 1.0f, dotProduct(wi, h));
        float NoH = clamp(0.0f, 1.0f, dotProduct(N, h));

        // Fresnel reflectance
        // Schlick 1994
        Vector3f F = Ks + (Vector3f(1.0f) - Ks) * std::pow(1.0f - LoH, 5.0f);

        // geometrical attenuation factor
        // Heitz 2014
        float lambdaV = NoL * std::sqrt(NoV * NoV * (1.0f - alphaSq) + alphaSq);
        float lambdaL = NoV * std::sqrt(NoL * NoL * (1.0f - alphaSq) + alphaSq);
        float V = 0.5f / (lambdaV + lambdaL);

        // normal distribution function
        // Trowbridge-Reitz 1975 (GGX 2007)
        float D = D_GGX(alpha, NoH);

        // Vector3f specular = F * G * D / (4.0f * dotProduct(wo, N) * dotProduct(wi, N));
        Vector3f specular = D * V * F;

        return specular;
        break;
    }
    }
}

#endif // RAYTRACING_MATERIAL_H
