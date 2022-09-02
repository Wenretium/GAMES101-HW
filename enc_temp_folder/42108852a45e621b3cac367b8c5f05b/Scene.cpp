//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0, 0.0, 0.0);
    }
    // 求光线与场景的交点
    Intersection inter_p = Scene::intersect(ray);
    if (!inter_p.happened)
        return Vector3f(0.0, 0.0, 0.0);
    Vector3f p = inter_p.coords;
    Vector3f N = inter_p.normal;
    Material* m = inter_p.m;
    
    // TO DO Implement Path Tracing Algorithm here
    // 交点是光源（想用物体obj来判断和提取，发现不行，一定要用材料m）
    if (m->hasEmission())
        return m->getEmission();
    Vector3f wo = ray.direction;
    // 在场景的所有光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度
    Intersection inter;
    float pdf_light; // pdf_light = 1 / A
    sampleLight(inter, pdf_light);
    // Get x, ws, NN
    Vector3f x = inter.coords;
    Vector3f ws = (x - p).normalized(); // 待渲染点到光源的采样点
    Vector3f NN = inter.normal;
    Vector3f emit = inter.emit;
    // Shoot a ray from p to x
    // If the ray is not blocked in the middle
    Intersection block = Scene::intersect(Ray(p, ws));
    Vector3f L_dir;
    //if (!block.happened) // 开始以为inter也记录了光源的物体，判断撞到的物体是不是光源。后来才发现光源不算一个物体。
    // 上面的判断语句是错误的，会导致整个场景都渲染不出来！！因为光线并不会和光源相撞，而继续运动跟光源后面的物体相撞了，进不去条件语句
    if(block.distance - (x - p).norm() > -0.001) // 有一个容错范围，否则整体较暗且有黑色横纹
    {
        // 直接光照
        L_dir = emit * m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN)\
            / dotProduct(x - p, x - p) / pdf_light;
    }
    Vector3f L_indir;
    // Test Russian Roulette with probability RussianRoulette
    if (get_random_float() > RussianRoulette)
        return L_dir; // 不计算间接光照了，递归出口
    Vector3f wi = m->sample(wo, N); // 按照该材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
    // Trace a ray r(p, wi)
    block = intersect(Ray(p, wi));
    // If ray r hit a non-emitting object at q
    if (block.happened && !block.m->hasEmission())
    {
        // 间接光照
        float pdf = m->pdf(wo, wi, N);
        if(pdf > 0.05) // pdf太小，造成渲染结果噪点过多（呈现白色）
            L_indir = castRay(Ray(p, wi), 0) * m->eval(wo, wi, N) * dotProduct(wi, N)\
            / pdf / RussianRoulette;
    }
    //std::cout << L_dir << "  ||||||  " << L_indir << std::endl;
    return L_dir + L_indir;
}