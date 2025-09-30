#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include "coord_utils.hpp"
#include <fstream>   //  파일 출력용

// MORAI에서 제공하는 오프셋
const double eastOffset  = 302459.942;
const double northOffset = 4122635.537;

// ENU 기준점
bool origin_set = false;
double origin_lat, origin_lon, origin_alt;

//  파일 출력 스트림
std::ofstream gps_file("results/output4.txt");   // GPS → ENU
std::ofstream ego_file("results/output3.txt");   // EGO → ENU

void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg)
{
    if (!origin_set) {
        origin_lat = msg->latitude;
        origin_lon = msg->longitude;
        origin_alt = msg->altitude;
        origin_set = true;

        // ROS_INFO("[INIT] ENU Origin set: lat=%.8f, lon=%.8f, h=%.3f",
        //          origin_lat, origin_lon, origin_alt);
    }

    // WGS84 → ECEF
    double x, y, z;
    wgs84ToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // ECEF → ENU
    double enu_x, enu_y, enu_z;
    ecefToENU(x, y, z, origin_lat, origin_lon, origin_alt, enu_x, enu_y, enu_z);

    // 파일 출력
    gps_file << enu_x << " " << enu_y << " " << enu_z << std::endl;
}

void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    if (!origin_set) {
        ROS_WARN("[EGO] Origin not set yet. Waiting for GPS...");
        return;
    }

    // 1. UTM 좌표 복원
    double ego_e = msg->position.x + eastOffset;
    double ego_n = msg->position.y + northOffset;
    double ego_h = msg->position.z;

    // 2. UTM → WGS84
    int zone = 52;
    bool northp = true;
    double lat, lon;
    utmToWgs84(ego_e, ego_n, zone, northp, lat, lon);

    // 3. WGS84 → ECEF
    double x, y, z;
    wgs84ToECEF(lat, lon, ego_h, x, y, z);

    // 4. ECEF → ENU
    double enu_x, enu_y, enu_z;
    ecefToENU(x, y, z, origin_lat, origin_lon, origin_alt, enu_x, enu_y, enu_z);

    // // 터미널 출력
    // ROS_INFO("[EGO] ENU: E=%.3f, N=%.3f, U=%.3f", enu_x, enu_y, enu_z);

    // 파일 출력
    ego_file << enu_x << " " << enu_y << " " << enu_z << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coord_transform_node");
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe("/gps", 10, gpsCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 10, egoCallback);

    ros::spin();

    // 종료 시 파일 닫기
    gps_file.close();
    ego_file.close();

    return 0;
}
