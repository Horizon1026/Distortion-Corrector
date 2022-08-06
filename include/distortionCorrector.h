#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class DistortionCorrectorClass {
private:
    //定义畸变参数
    double k1 = -0.28340811;
    double k2 = 0.07395907;
    double k3 = 0.0;
    double p1 = 0.00019359;
    double p2 = 1.76187114e-05;

    //定义相机内参
    double fx = 458.654;
    double fy = 457.296;
    double cx = 367.215;
    double cy = 248.375;

private:
    //定义反畸变映射表
    int mapScale = 1;
    std::vector<std::vector<std::pair<float, float>>> distortedToCorrectMap;

public:
    /* 不带参数的构造函数 */
    DistortionCorrectorClass();

    /* 带参数的构造函数 */
    DistortionCorrectorClass(double SET_k1, double SET_k2, double SET_k3, double SET_p1, double SET_p2,
                             double SET_fx, double SET_fy, double SET_cx, double SET_cy);

    /* 设置畸变参数 */
    void SetDistortionParameters(double SET_k1, double SET_k2, double SET_k3, double SET_p1, double SET_p2);

    /* 设置相机内参 */
    void SetCameraIntrinsicsMatrix(double SET_fx, double SET_fy, double SET_cx, double SET_cy);

    /* 初始化反畸变映射表 */
    bool InitUndistortedMap(int map_scale, int rows, int cols);

    /* 由非畸变像素平面坐标，变化为畸变像素平面坐标 */
    cv::Point2f Undistortion2Distortion(cv::Point2f p);

    /* 由畸变像素平面坐标，变化为非畸变像素平面坐标 */
    cv::Point2f Distortion2Undistortion(cv::Point2f p);
};