#include <distortionCorrector.h>
#include <FASTFeatureDetector.h>
#include <string>
#include <iostream>

//定义图片存放路径
std::string image_filepath = "../examples/distorted.png";

int main() {
    // 初始化 FAST 角点检测器
    FASTFeatureDetectorClass FASTFeatureDetector(0.2, 10, 9, 30, 10);

    // 加载图像（以灰度方式），并检查是否加载成功，如果不成功则终止程序
    cv::Mat image0 = cv::imread(image_filepath, 0);
    assert(image0.data != nullptr);

    // 检测图像中的 FAST 角点
    std::vector<cv::Point2f> points = FASTFeatureDetector.DetectGoodSparseFeatures(image0);
    // 在原图上画上特征点
    cv::Mat showImage0 = cv::Mat(image0.rows, image0.cols, CV_8UC3);
    cv::cvtColor(image0, showImage0, CV_GRAY2BGR);
    for (auto &point : points) {
        cv::circle(showImage0, point, 3, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("image0 with FAST features", showImage0);

    // 初始化畸变校正器
    DistortionCorrectorClass DistortionCorrector;
    DistortionCorrector.InitUndistortedMap(5, image0.rows, image0.cols);

    // 矫正畸变图像
    cv::Mat image1 = cv::Mat(image0.rows, image0.cols, CV_8UC1);
    for (unsigned int u = 0; u < image1.cols; u++) {
        for (unsigned int v = 0; v < image1.rows; v++) {
            cv::Point2f uvc = cv::Point2f(u, v);
            cv::Point2f uvd = DistortionCorrector.Undistortion2Distortion(uvc);
            image1.at<uchar>(v, u) = image0.at<uchar>(int(uvd.y), int(uvd.x));
        }
    }

    // 在矫正图上画上特征点
    cv::Mat showImage1 = cv::Mat(image1.rows, image1.cols, CV_8UC3);
    cv::cvtColor(image1, showImage1, CV_GRAY2BGR);
    for (auto &point : points) {
        cv::circle(showImage1, DistortionCorrector.Distortion2Undistortion(point), 3, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("image1 with FAST features", showImage1);

    // 暂停
    cv::waitKey();

    return 0;
}