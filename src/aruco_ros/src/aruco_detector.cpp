#include "aruco_cvversioning.h"
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "aruco.h"
#include <Eigen/Geometry>

#include "dcf/dcfmarkertracker.h"

#include <ros/ros.h>
#include <duckietown_msgs/AprilTagExtended.h>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/aruco.hpp"

using namespace std;
using namespace aruco;

struct TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;

    TimerAvrg(int _n = 30) {
        n = _n;
        times.reserve(n);
    }

    inline void start() { begin = std::chrono::high_resolution_clock::now(); }

    inline void stop() {
        end = std::chrono::high_resolution_clock::now();
        double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
        if (times.size() < n) times.push_back(duration);
        else {
            times[curr] = duration;
            curr++;
            if (curr >= times.size()) curr = 0;
        }
    }

    double getAvrg() {
        double sum = 0;
        for (auto t:times) sum += t;
        return sum / double(times.size());
    }
};

TimerAvrg timer;
TimerAvrg timer_all;

DFCMarkerTracker TheTracker;

void toQuat(const cv::Mat &rvec, geometry_msgs::Quaternion &quat) {
    double x = rvec.at<float>(0);
    double y = rvec.at<float>(1);
    double z = rvec.at<float>(2);
    double r = sqrt(x * x + y * y + z * z);
    double c = cos(r / 2);
    double s = sin(r / 2);
    quat.x = c;
    quat.y = s * z / r;
    quat.z = -s * y / r;
    quat.w = -s * x / r;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco detector");
    ros::NodeHandle node_handle("~");

    ros::Publisher poses_pub = node_handle.advertise<duckietown_msgs::AprilTagExtended>("/poses_acquisition/poses", 1);


    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile("/aruco_ros_docker/src/aruco_ros/cam_calibr.yml");

    float MarkerSize = 0.065;

    cv::VideoCapture video;
    video.open("http://192.168.1.58:8080/video");

    if (!video.isOpened()) {
        throw std::runtime_error("Could not open video");
    }

    cv::Mat image;

    int frameid = 0;

    video.set(CV_CAP_PROP_POS_FRAMES, frameid);
    while (image.empty()) {
        video >> image;
    }


    TheTracker.setDictionary("TAG36h11");

    if (CamParam.isValid() && MarkerSize != -1) {
        CamParam.resize(image.size());
        TheTracker.setParams(CamParam, MarkerSize);
    }

    do {
        video.retrieve(image);

        timer.start();
        timer_all.start();
        std::map<int, cv::Ptr<TrackerImpl>> setTrackers = TheTracker.track(image);
        TheTracker.estimatePose();
        timer.stop();

        for (const auto& m:setTrackers) {
            Marker marker = m.second->getMarker();
            std::cout << "                       " << marker << " W=" << m.second->getTrustVal() << std::endl;
            if (marker.Tvec.empty() || marker.Rvec.empty()) {
                continue;
            }

            duckietown_msgs::AprilTagExtended ate_msg;
            ate_msg.header.frame_id = "watchtower_66";
            ate_msg.header.stamp = ros::Time::now();
            ate_msg.tag_id = marker.id;

            geometry_msgs::Quaternion rot;
            toQuat(marker.Rvec, rot);

            geometry_msgs::Vector3 trl;
            trl.x = marker.Tvec.at<float>(0);
            trl.y = marker.Tvec.at<float>(1);
            trl.z = marker.Tvec.at<float>(2);

            geometry_msgs::Transform trf;
            trf.rotation = rot;
            trf.translation = trl;

            ate_msg.transform = trf;

            poses_pub.publish(ate_msg);
        }

//             std::cout <<"|@ Frame:"<<frameid++<<"/"+to_string(maxNFrames)<<", fps:"<<1./timer.getAvrg() << std::endl;
        timer_all.stop();
        std::cout << "fps:" << 1.0 / timer.getAvrg() << " : " <<  1.0 / timer_all.getAvrg() << std::endl;

//        TheTracker.draw(image);

//        cv::resize(image, image, cv::Size(640, 480));
//        cv::imshow("image", image);
//        cv::waitKey(1);

    } while (video.grab());
}
