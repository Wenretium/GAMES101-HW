// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

bool SameSide(Vector3f A, Vector3f B, Vector3f C, Vector3f P)
{
    Vector3f AB = B - A;
    Vector3f AC = C - A;
    Vector3f AP = P - A;

    Vector3f v1 = AB.cross(AC);
    Vector3f v2 = AB.cross(AP);

    // v1 and v2 should point to the same direction
    return v1.dot(v2) >= 0;
}

// 这里踩了个坑！在做super-sampling时，发现四个小像素格要不全在三角形内，要不全在三角形外，无法做到count=1/2/3
// 是因为这个函数的定义（原作业框架）中，传进来的xy是整型
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P(x, y, 0);
    return SameSide(_v[0], _v[1], _v[2], P) &&
        SameSide(_v[1], _v[2], _v[0], P) &&
        SameSide(_v[2], _v[0], _v[1], P);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
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

float min(float a, float b, float c)
{
    float min_value = a;
    if (b < min_value)min_value = b;
    if (c < min_value)min_value = c;
    return min_value;
}

float max(float a, float b, float c)
{
    float max_value = a;
    if (b > max_value)max_value = b;
    if (c > max_value)max_value = c;
    return max_value;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // 创建三角形的 2 维 bounding box
    int min_x = (int)std::min(v[0][0], std::min(v[1][0], v[2][0]));
    int max_x = (int)std::max(v[0][0], std::max(v[1][0], v[2][0]));
    int min_y = (int)std::min(v[0][1], std::min(v[1][1], v[2][1]));
    int max_y = (int)std::max(v[0][1], std::max(v[1][1], v[2][1]));
    // 遍历此 bounding box 内的所有像素
    std::vector<Eigen::Vector2f> pos
    {
        {0.25,0.25},
        {0.75,0.25},
        {0.25,0.75},
        {0.75,0.75},
    }; // super-sampling
    for (int x = min_x; x <= max_x; x++)
        for (int y = min_y; y <= max_y; y++)
        {
            int count = 0;
            int max_count = 4; // 2*2
            float min_depth = std::numeric_limits<float>::infinity();
            for (int i = 0; i < 4; i++)
            {
                if (insideTriangle(x + pos[i][0], y + pos[i][1], t.v)) // 此像素在三角形内部
                {
                    count++;
                    // 计算插值深度值 
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + pos[i][0], y + pos[i][1], t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated < min_depth)
                        min_depth = z_interpolated; // 插值深度的最小值，如果通过深度检验，就要着色
                }
            }
            if (count > 0)
            {
                if (min_depth < depth_buf[get_index(x, y)])
                {
                    set_pixel(Vector3f(x, y, min_depth), t.getColor()*(count/4.0)+((max_count-count)/4.0)*frame_buf[get_index(x, y)]);
                    depth_buf[get_index(x, y)] = min_depth;
                }
                if (count < max_count) // 清除depth_buf，使得较远的图形能重复着色这个像素点
                {
                    depth_buf[get_index(x, y)] = std::numeric_limits<float>::infinity();
                }
            }
        }

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on