//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    //void Render(const Scene& scene, std::vector<Vector3f>& framebuffer, int begin_height, int height);
    void MultiThreadRender(const Scene& scene);

private:
};
