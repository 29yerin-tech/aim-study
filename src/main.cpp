#include <ros/ros.h>                       // ROS 기본 기능
#include <sensor_msgs/NavSatFix.h>         // /gps/fix 메시지 타입 (위도/경도/고도)
#include <morai_msgs/EgoVehicleStatus.h>   // /Ego_topic 메시지 타입 (차량 상태/위치)
#include <geodesy/utm.h>                   // WGS84 → UTM 변환 도우미
#include <geographic_msgs/GeoPoint.h>      // 위도/경도/고도 구조체
#include <fstream>                         // ofstream: 파일 출력 스트림
#include <iomanip>                         // fixed, setprecision 등 포맷 설정
#include <morai_msgs/GPSMessage.h>

using namespace std;                       // std:: 생략용 (필수는 아님)

const double eastOffset = 302459.942;
const double northOffset = 4122635.537;

// 두 개의 출력 파일 스트림을 전역으로 열어둠
ofstream gps_file("output2.txt");          // GPS(변환된 UTM) 저장용
ofstream ego_file("output1.txt");          // Ego(원본 UTM) 저장용

// ── GPS 콜백: /gps/fix 수신 → WGS84를 UTM으로 변환 → output2.txt에 기록
void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg)
{
     geographic_msgs::GeoPoint geo;         // 변환 입력 구조체
     geo.latitude  = msg->latitude;         // 위도
     geo.longitude = msg->longitude;        // 경도
     geo.altitude  = msg->altitude;         // 고도

     geodesy::UTMPoint utm(geo);            // 변환 수행 (Easting/ Northing/ Zone 등 계산됨)
     cout << "East : " << utm.easting - eastOffset<< " North : " << utm.northing - northOffset << endl;

     if (gps_file.is_open()) {              // 파일이 정상 열렸다면
          gps_file << fixed << setprecision(3)
                   << utm.easting - eastOffset << " " << utm.northing - northOffset << "\n";  // 소수점 3자리로 저장
         // "\n"은 줄바꿈(빠름). endl은 줄바꿈+즉시 flush(느릴 수 있음)
     }
}




// ── Ego 콜백: /Ego_topic 수신 → 원본 UTM 좌표를 output1.txt에 기록
void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    double ego_x = msg->position.x;        // UTM X (Easting)
    double ego_y = msg->position.y;        // UTM Y (Northing)

    if (ego_file.is_open()) {
        ego_file << fixed << setprecision(3)
                 << ego_x << " " << ego_y << "\n";               // 파일에 저장
    }

    // 필요하면 터미널에도 보고:
    // cout << ego_x << " " << ego_y << endl;   // 화면 확인용(선택)
}

// ── main: 구독자 연결 후 콜백만 돌리며 대기
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_and_ego_logger");   // 노드 초기화 (이름: gps_and_ego_logger)
    ros::NodeHandle nh;                             // 노드 핸들

    // 토픽 구독 연결
    ros::Subscriber gps_sub = nh.subscribe("/gps",   10, gpsCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 10, egoCallback);

    ros::spin();                                    // 콜백 무한 처리 (별도 루프 불필요)

    // 노드 종료 시 파일 닫기
    gps_file.close();
    ego_file.close();
    return 0;
}
