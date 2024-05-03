#include "BVH.hpp"
#include <algorithm>
#include <cassert>

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
    SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode))
    , splitMethod(splitMethod)
    , primitives(std::move(p))
{
    printf("Using SplitMethod %s\n", splitMethod == SplitMethod::NAIVE ? "NAIVE" : "SAH");
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector { objects[0] });
        node->right = recursiveBuild(std::vector { objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();

        auto mid = objects.begin() + (objects.size() / 2);
        switch (splitMethod) {
        case SplitMethod::NAIVE:
            std::nth_element(objects.begin(), mid, objects.end(), [dim](auto f1, auto f2) {
                return f1->getBounds().Centroid()[dim] < f2->getBounds().Centroid()[dim];
            });
            break;
        case SplitMethod::SAH:
        default:
            // Allocate BucketInfo for SAH partition buckets
            constexpr int nBuckets = 12;
            struct BucketInfo {
                int count = 0;
                Bounds3 bounds;
            };
            BucketInfo buckets[nBuckets];

            // Initialize BucketInfo
            for (Object* object : objects) {
                int b = nBuckets * centroidBounds.Offset(object->getBounds().Centroid())[dim];
                b = std::min(b, nBuckets - 1);
                buckets[b].count++;
                buckets[b].bounds = Union(buckets[b].bounds, object->getBounds());
            }

            // Find bucket to split at that minimizes SAH
            float minCost = std::numeric_limits<float>::max();
            int minCostSplitBucket = 0;
            for (int i = 0; i < nBuckets - 1; ++i) {
                // Compute cost of splitting after bucket i
                Bounds3 boundsL, boundsR;
                int countL = 0, countR = 0;
                for (int j = 0; j <= i; ++j) {
                    boundsL = Union(boundsL, buckets[j].bounds);
                    countL += buckets[j].count;
                }
                for (int j = i + 1; j < nBuckets; ++j) {
                    boundsR = Union(boundsR, buckets[j].bounds);
                    countR += buckets[j].count;
                }
                float cost = countL * boundsL.SurfaceArea() + countR * boundsR.SurfaceArea();
                if (cost < minCost) {
                    minCost = cost;
                    minCostSplitBucket = i;
                }
            }

            // Split at minCostSplitBucket
            mid = std::partition(objects.begin(), objects.end(), [=](auto object) {
                int b = nBuckets * centroidBounds.Offset(object->getBounds().Centroid())[dim];
                b = std::min(b, nBuckets - 1);
                return b <= minCostSplitBucket;
            });
            break;
        }

        auto leftshapes = std::vector<Object*>(objects.begin(), mid);
        auto rightshapes = std::vector<Object*>(mid, objects.end());

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsNeg { int(ray.direction.x > 0),
        int(ray.direction.y > 0),
        int(ray.direction.z > 0) };
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return Intersection();
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);
    return hit1.distance < hit2.distance ? hit1 : hit2;
}