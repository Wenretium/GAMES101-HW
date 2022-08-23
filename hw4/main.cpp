#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
int num_control_points = 6; // 修改控制点数量

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < num_control_points)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size() == 1)
        return control_points[0];

    std::vector<cv::Point2f> new_control_points;
    for (int i = 0; i < control_points.size() - 1; i++)
        new_control_points.push_back((1-t)*control_points[i]+t*control_points[i+1]);

    cv::Point2f p = recursive_bezier(new_control_points, t);

    return p;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        //window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        // 反走样
        // 1.计算四角的四个像素点
        cv::Point2f p1 = cv::Point2f(point.x - floor(point.x) <= 0.5 ? floor(point.x)-1: floor(point.x),\
            point.y - floor(point.y) <= 0.5 ? floor(point.y) - 1 : floor(point.y));
        cv::Point2f p2 = cv::Point2f(p1.x + 1, p1.y);
        cv::Point2f p3 = cv::Point2f(p1.x, p1.y + 1);
        cv::Point2f p4 = cv::Point2f(p1.x + 1, p1.y + 1);
        // 2.计算点point分别到这四个像素中心点的距离
        float dist1 = sqrt(pow(point.x - (p1.x + 0.5), 2) + pow(point.y - (p1.y + 0.5), 2));
        float dist2 = sqrt(pow(point.x - (p2.x + 0.5), 2) + pow(point.y - (p2.y + 0.5), 2));
        float dist3 = sqrt(pow(point.x - (p3.x + 0.5), 2) + pow(point.y - (p3.y + 0.5), 2));
        float dist4 = sqrt(pow(point.x - (p4.x + 0.5), 2) + pow(point.y - (p4.y + 0.5), 2));
        // 3.计算权重，越近则权重越大
        // sqrt(2)为最大距离
        float w1 = sqrt(2) - dist1;
        float w2 = sqrt(2) - dist2;
        float w3 = sqrt(2) - dist3;
        float w4 = sqrt(2) - dist4;
        float sum_w = w1 + w2 + w3 + w4;
        w1 /= sum_w;
        w2 /= sum_w;
        w3 /= sum_w;
        w4 /= sum_w;
        // 4.根据权重给像素赋值
        // 每个像素点在原来基础上加上新颜色（否则颜色很浅，因为有多个点影响一个像素），且值不超出255
        window.at<cv::Vec3b>(p1.y, p1.x)[1] = std::min(255.f, window.at<cv::Vec3b>(p1.y, p1.x)[1] + 255 * w1);
        window.at<cv::Vec3b>(p2.y, p2.x)[1] = std::min(255.f, window.at<cv::Vec3b>(p2.y, p2.x)[1] + 255 * w2);
        window.at<cv::Vec3b>(p3.y, p3.x)[1] = std::min(255.f, window.at<cv::Vec3b>(p3.y, p3.x)[1] + 255 * w3);
        window.at<cv::Vec3b>(p4.y, p4.x)[1] = std::min(255.f, window.at<cv::Vec3b>(p4.y, p4.x)[1] + 255 * w4);

    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == num_control_points)
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
