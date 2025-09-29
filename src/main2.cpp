#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>          // /gps 데이터
#include <morai_msgs/EgoVehicleStatus.h>    // /Ego_topic 데이터
#include <fstream>                          // 파일 저장
#include <iomanip>                          // 출력 포맷
#include "coord_utils.hpp"                  // 좌표 변환 함수 선언

using namespace std;

// ───── 모라이 심 오프셋 ─────
const double eastOffset  = 302459.942;
const double northOffset = 4122635.537;

// ───── 기준점(origin) 관련 변수 ─────
bool origin_set = false;
double lat0, lon0, h0;
double x0, y0, z0;

// ───── 출력 파일 ─────
ofstream gps_file("results/output4.txt");   // GPS → ENU
ofstream ego_file("results/output3.txt");   // Ego → ENU

// ───── GPS 콜백 ─────
void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    double x, y, z;
    wgs84ToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    if (!origin_set) {
        origin_set = true;
        lat0 = msg->latitude;
        lon0 = msg->longitude;
        h0   = msg->altitude;
        wgs84ToECEF(lat0, lon0, h0, x0, y0, z0);
    }

    double e, n, u;
    ecefToENU(x, y, z, x0, y0, z0, lat0, lon0, e, n, u);
    gps_file << fixed << setprecision(3) << e << " " << n << " " << u << "\n";
}

// ───── Ego 콜백 ─────
void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    if (!origin_set) {
        return;  // 아직 origin이 없으면 무시
    }

    double ego_e = msg->position.x + eastOffset;
    double ego_n = msg->position.y + northOffset;
    double h     = msg->position.z;

    double lat, lon;
    utmToWgs84(ego_e, ego_n, 52, true, lat, lon);

    double x, y, z;
    wgs84ToECEF(lat, lon, h, x, y, z);

    double e, n, u;
    ecefToENU(x, y, z, x0, y0, z0, lat0, lon0, e, n, u);
    ego_file << fixed << setprecision(3) << e << " " << n << " " << u << "\n";
}

// ───── main ─────
int main(int argc, char** argv) {
    ros::init(argc, argv, "coord_logger");  // 노드 이름
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe("/gps", 10, gpsCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 10, egoCallback);

    ros::spin();

    gps_file.close();
    ego_file.close();
    return 0;
}
