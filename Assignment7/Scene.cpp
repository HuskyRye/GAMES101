//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = intersect(ray);
    Material* m = intersection.m;
    if (!intersection.happened)
        return Vector3f(0.0f);
    if (m->hasEmission()) {
        if (depth == 0)
            return m->getEmission();
        else
            return Vector3f(0.0f);
    }

    Vector3f L_dir, L_indir;
    Vector3f wo = -ray.direction;
    Vector3f p = intersection.coords;
    Vector3f N = intersection.normal;

    // Direct illumination
    Intersection light;
    float pdf_light;
    sampleLight(light, pdf_light);
    Vector3f x = light.coords;
    Vector3f ws = x - p;
    float lightDistance = ws.norm();
    ws = normalize(ws);
    if (intersect(Ray(p, ws)).distance > lightDistance - EPSILON) {
        float cos_theta = std::max(0.f, dotProduct(ws, N));
        float cos_theta_x = std::max(0.f, dotProduct(-ws, light.normal));
        L_dir = light.emit * m->eval(wo, ws, N) * cos_theta * cos_theta_x
            / lightDistance / lightDistance / pdf_light;
    }

    // Indirect illumination
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = normalize(m->sample(wo, N));
        float pdf_hemi = m->pdf(wo, wi, N);
        if (pdf_hemi > EPSILON) { // Otherwise, it will cause white spots
            float cos_theta = std::max(0.f, dotProduct(wi, N));
            L_indir = castRay(Ray(p, wi), depth + 1) * m->eval(wo, wi, N) * cos_theta
                / pdf_hemi / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}