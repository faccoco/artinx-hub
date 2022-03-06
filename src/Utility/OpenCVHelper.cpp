#include "Utility.hpp"

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color) {
    cv::Point2f pts[5];
    rect.points(pts);
    pts[4] = pts[0];

    for(int i = 0; i < 4; ++i)
        cv::line(img, cv::Point{ pts[i] }, cv::Point{ pts[i + 1] }, color, 1);
}
