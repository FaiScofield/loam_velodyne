#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{
    cv::Mat imgLeft_ = cv::imread(argv[1], 0);
    cv::Mat imgRight_ = cv::imread(argv[2], 0);

    if (imgLeft_.empty() || imgRight_.empty())
        return -1;

   cv::imshow("first image", imgLeft_);
   cv::imshow("second image", imgRight_);
   cv::waitKey(0);


    // 计算左右图像的ORB特征点并匹配

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000, 1.2, 8);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create(1000, 1.2, 8);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    cv::Mat descLeft_, descRight_;
    vector<cv::KeyPoint> kpsLeft_, kpsRight_;
    std::vector<cv::DMatch> matches_1st, matches_2nd, matches_;

    detector->detect(imgLeft_, kpsLeft_);
    detector->detect(imgRight_, kpsRight_);
    descriptor->compute(imgLeft_, kpsLeft_, descLeft_);
    descriptor->compute(imgRight_, kpsRight_, descRight_);
    matcher->match(descLeft_, descRight_, matches_1st);

    cv::Mat image_show;
    cv::drawMatches(imgLeft_, kpsLeft_, imgRight_, kpsRight_, matches_1st, image_show);
    cv::imshow("matched before ransac", image_show);
    cv::waitKey(0);


    // 找到正确的匹配
    double min_dist = 10000;
    for (auto& m : matches_1st ) {
        double dist = m.distance;
        if (dist < min_dist) { min_dist = dist; }
    }
    for (auto& m : matches_1st ) {
        if ( m.distance <= std::max(2 * min_dist, 20.0) ) {
            matches_2nd.push_back(m);
        }
    }

    cv::drawMatches(imgLeft_, kpsLeft_, imgRight_, kpsRight_, matches_2nd, image_show);
    cv::imshow("matched before ransac 2", image_show);
    cv::waitKey(0);



    // RANSAC 再次消除误匹配
    vector<KeyPoint> kpsLeft_2nd, kpsRight_2nd;
    vector<Point2f> kpl, kpr;
    Mat descLeft_2nd;
    for (size_t i=0; i<matches_2nd.size(); i++) {
       // 经过此步 kpsLeft_2nd 和 kpsRight_2nd 在顺序上有一一匹配关系了
       kpsLeft_2nd.push_back(kpsLeft_[matches_2nd[i].queryIdx]);
       kpsRight_2nd.push_back(kpsRight_[matches_2nd[i].trainIdx]);
       kpl.push_back(kpsLeft_2nd[i].pt);
       kpr.push_back(kpsRight_2nd[i].pt);
       descLeft_2nd.push_back(descLeft_.row(matches_2nd[i].queryIdx).clone());
    }

    vector<uchar> RansacStatus;
    Mat H = findHomography(kpl, kpr, CV_RANSAC, 3, RansacStatus);

    vector<KeyPoint> aft_ran_kpsl, aft_ran_kpsr;
    Mat aft_ran_descl;
    int index = 0;
    for (size_t i=0; i<matches_2nd.size(); i++) {
       if (RansacStatus[i] != 0) {
           aft_ran_kpsl.push_back(kpsLeft_2nd[i]);
           aft_ran_kpsr.push_back(kpsRight_2nd[i]);
           matches_2nd[i].queryIdx = index;
           matches_2nd[i].trainIdx = index;
           matches_.push_back(matches_2nd[i]);  // 匹配的索引基于aft_ran_kps
           aft_ran_descl.push_back(descLeft_2nd.row(i).clone());
           index++;
       }
    }


    // 更新特征点和对应的描述子成员变量
    kpsLeft_.swap(aft_ran_kpsl);
    kpsRight_.swap(aft_ran_kpsr);
    descLeft_ = aft_ran_descl.clone();

    cv::drawMatches(imgLeft_, kpsLeft_, imgRight_, kpsRight_, matches_, image_show);
    cv::imshow("matched after ransac", image_show);
    cv::waitKey(0);

    // 显示左右两幅图的匹配情况
    cv::Mat img1_show, img2_show;
    cv::cvtColor(imgLeft_, img1_show, CV_GRAY2BGR);
    cv::cvtColor(imgRight_, img2_show, CV_GRAY2BGR);
    cv::Mat img_match_show(2*img1_show.rows, img1_show.cols, CV_8UC3);
    img1_show.copyTo(img_match_show(cv::Rect(0, 0, img1_show.cols, img1_show.rows)));
    img2_show.copyTo(img_match_show(cv::Rect(0, img1_show.rows, img2_show.cols, img2_show.rows)));

    for (auto &m : matches_) {
        cv::circle(img_match_show, cv::Point2d(kpsLeft_[m.queryIdx].pt.x, kpsLeft_[m.queryIdx].pt.y), 3, cv::Scalar(0,255,0), 2);
        cv::circle(img_match_show, cv::Point2d(kpsRight_[m.trainIdx].pt.x, kpsRight_[m.trainIdx].pt.y+imgLeft_.rows), 3, cv::Scalar(0,255,0), 2);
        cv::line(img_match_show, cv::Point2d(kpsLeft_[m.queryIdx].pt.x, kpsLeft_[m.queryIdx].pt.y), cv::Point2d(kpsRight_[m.trainIdx].pt.x, kpsRight_[m.trainIdx].pt.y+imgLeft_.rows), cv::Scalar(0,255,0), 1);

    }
    cv::imshow("matched after ransac", img_match_show);
    cv::waitKey(1);

}