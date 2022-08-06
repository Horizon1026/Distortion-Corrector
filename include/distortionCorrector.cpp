#include <distortionCorrector.h>


/* 不带参数的构造函数 */
DistortionCorrectorClass::DistortionCorrectorClass() {
    //定义畸变参数
    k1 = -0.28340811;
    k2 = 0.07395907;
    k3 = 0.0;
    p1 = 0.00019359;
    p2 = 1.76187114e-05;

    //定义相机内参
    fx = 458.654;
    fy = 457.296;
    cx = 367.215;
    cy = 248.375;
}


/* 带参数的构造函数 */
DistortionCorrectorClass::DistortionCorrectorClass(double SET_k1, double SET_k2, double SET_k3,
    double SET_p1, double SET_p2, double SET_fx, double SET_fy, double SET_cx, double SET_cy) {
    SetDistortionParameters(SET_k1, SET_k2, SET_k3, SET_p1, SET_p2);
    SetCameraIntrinsicsMatrix(SET_fx, SET_fy, SET_cx, SET_cy);
}


/* 设置畸变参数 */
void DistortionCorrectorClass::SetDistortionParameters(double SET_k1, double SET_k2, double SET_k3, double SET_p1, double SET_p2) {
    k1 = SET_k1;
    k2 = SET_k2;
    k3 = SET_k3;
    p1 = SET_p1;
    p2 = SET_p2;
}


/* 设置相机内参 */
void DistortionCorrectorClass::SetCameraIntrinsicsMatrix(double SET_fx, double SET_fy, double SET_cx, double SET_cy) {
    fx = SET_fx;
    fy = SET_fy;
    cx = SET_cx;
    cy = SET_cy;
}


/* 初始化反畸变映射表 */
bool DistortionCorrectorClass::InitUndistortedMap(int map_scale, int rows, int cols) {
    //初始化内存空间
    int mapCol = cols * map_scale;
    int mapRow = rows * map_scale;
    std::vector<std::pair<float, float>> line(mapCol, std::pair<float, float>(INFINITY, INFINITY));
    for (int i = 0; i < mapRow; i++) {
        distortedToCorrectMap.emplace_back(line);
    }

    //由正确的点坐标，计算畸变的点坐标
    float step = 1.0 / float(map_scale);
    for (float vc = - rows / 2; vc < rows + rows / 2; vc+=step) {
        for (float uc = - cols / 2; uc < cols + cols / 2; uc+=step) {
            cv::Point2f pc = cv::Point2f(uc, vc);
            cv::Point2f pd = Undistortion2Distortion(pc);
            int ud = int(pd.x * map_scale);
            int vd = int(pd.y * map_scale);
            if (pd.x - ud > 0.5) {
                ud++;
            }
            if (pd.y - vd > 0.5) {
                vd++;
            }
            if (vd > -1 && vd < mapRow && ud > -1 && ud < mapCol) {
                if (distortedToCorrectMap[vd][ud].first == INFINITY) {
                    distortedToCorrectMap[vd][ud].first = pc.x;
                    distortedToCorrectMap[vd][ud].second = pc.y;
                } else {
                    distortedToCorrectMap[vd][ud].first = (distortedToCorrectMap[vd][ud].first + pc.x) / 2;
                    distortedToCorrectMap[vd][ud].second = (distortedToCorrectMap[vd][ud].second + pc.y) / 2;
                }
            }
        }
    }

    //记录表比例尺
    mapScale = map_scale;
    return true;
}


/* 由非畸变像素平面坐标，变化为畸变像素平面坐标 */
cv::Point2f DistortionCorrectorClass::Undistortion2Distortion(cv::Point2f p) {
    //将输入的像素坐标转化到归一化平面上，p.x是u，p.y是v
    double x = (p.x - cx) / fx;
    double y = (p.y - cy) / fy;

    //根据畸变参数计算畸变后的归一化平面坐标
    cv::Point2f p_distored;
    double r_2 = x * x + y * y;     //计算r的平方
    p_distored.x = x * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + 2 * p1 * x * y + p2 * (r_2 + 2 * x * x);
    p_distored.y = y * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + 2 * p2 * x * y + p1 * (r_2 + 2 * y * y);

    //将畸变后的归一化平面坐标转化成像素坐标
    p_distored.x = fx * p_distored.x + cx;
    p_distored.y = fy * p_distored.y + cy;

    return p_distored;
}


/* 由畸变像素平面坐标，变化为非畸变像素平面坐标 */
cv::Point2f DistortionCorrectorClass::Distortion2Undistortion(cv::Point2f p_distort) {
    int ud = int(p_distort.x * mapScale);
    int vd = int(p_distort.y * mapScale);
    if (p_distort.x - ud > 0.5) {
        ud++;
    }
    if (p_distort.y - vd > 0.5) {
        vd++;
    }
    cv::Point2f p_correct = cv::Point2f(0, 0);
    if (vd > -1 && vd < (int)distortedToCorrectMap.size() && ud > -1 && ud < (int)distortedToCorrectMap[0].size()) {
        p_correct.x = distortedToCorrectMap[vd][ud].first;
        p_correct.y = distortedToCorrectMap[vd][ud].second;
    }

    return p_correct;
}