#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);
    //root = recursiveBuildSAH(primitives);

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
    if (objects.size() == 1) { // 使得每个叶子节点只包含一个物体
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent(); // 最长的轴
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

// SAH
BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) { // 使得每个叶子节点只包含一个物体
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{ objects[0] });
        node->right = recursiveBuild(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
        double S_N = centroidBounds.SurfaceArea();
        double min_cost = std::numeric_limits<float>::infinity();
        std::vector<Object*> best_leftshapes;
        std::vector<Object*> best_rightshapes;
        // 超参数
        int B = 4; // B < 32
        int C_trav = 1;
        int C_isect = 10;
        for (int dim = 0; dim < 3; dim++) // 对于每个轴
        {
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                    });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                    });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                    });
                break;
            }
            for (int j = 1; j < B; j++) // 等距离划分为B份
            {
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * j / B);
                // 这里踩了一个坑！我写的是 objects.size() / B * j，结果 objects.size() / B 自动类型为整数，
                // 如果objects.size() < B，middling取0，会导致左半没有划分到物体，后续计算出错，栈溢出。
                auto ending = objects.end();

                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);

                assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                // 计算花销
                Bounds3 centroidBoundsA;
                for (int i = 0; i < leftshapes.size(); ++i)
                    centroidBoundsA =
                    Union(centroidBoundsA, leftshapes[i]->getBounds().Centroid());
                double S_A = centroidBoundsA.SurfaceArea();
                Bounds3 centroidBoundsB;
                for (int i = 0; i < rightshapes.size(); ++i)
                    centroidBoundsB =
                    Union(centroidBoundsB, rightshapes[i]->getBounds().Centroid());
                double S_B = centroidBoundsB.SurfaceArea();
                double cost = C_trav + leftshapes.size() * S_A / S_N * C_isect + rightshapes.size() * S_B / S_N * C_isect;

                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_leftshapes = leftshapes;
                    best_rightshapes = rightshapes;
                }
            }
        }

        // 取花销最小的轴和分割平面
        node->left = recursiveBuildSAH(best_leftshapes);
        node->right = recursiveBuildSAH(best_rightshapes);

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
    Vector3f invDir (1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);
    std::array<int, 3> disIsNeg = {int(ray.direction.x < 0), int(ray.direction.y < 0), int(ray.direction.z < 0)};

    if (!node->bounds.IntersectP(ray, invDir, disIsNeg))
        return Intersection();

    if (node->left == nullptr && node->right == nullptr) // leaf node
        return node->object->getIntersection(ray);

    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    return hit1.distance < hit2.distance ? hit1:hit2;
}