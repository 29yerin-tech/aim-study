#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include "coord_utils.hpp"
#include <fstream>
#include <cmath>

namespace path_recorder {

// ── 웨이포인트 출력 파일 경로 ───────────────────────────
std::ofstream path_file("results/path.csv");

// ── ENU origin 및 마지막 저장점 ────────────────────────
bool   origin_set = false; // ENU 기준점 설정 여부
double lat0, lon0, h0; // ENU 기준점의 WGS84  (처음 ego 리스폰 위치 그니까 처음 리스폰 지점위도경도고도를 기준점으로 잡음)
double last_x = NAN, last_y = NAN;    // 직전에 저장한 ENU (x,y) , NAN: Not A Number (숫자가 아님, 초기값으로 사용)->아직 값이 없다는 뜻 그냥(0,0)으로 하면 실제좌표(0,0)랑 헷갈릴 수 있으니까..

// ── 콜백: /Ego_topic 수신 시 ENU 변환 + 0.1m 저장 ─────
void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {  
  ; // [1] 첫 메시지 → ENU origin 설정 여기 위에서 msg는 포인터(주소만 가르킴 메시지가 여기잇다..)
  if (!origin_set) {
    setOriginFromEgo(*msg, lat0, lon0, h0);   // 이때 모라이에서 주는 데이터는 utm-오프셋 임.. 좌표변환을 위해, 원본 데이터에 오프셋 더하고 시작해야하고 이때, offset 더하기는 coord_utils. cpp 내부에서 처리함
    origin_set = true;  //여기서 *msg는 포인터가 가르키는 객체, 메시지 안의 데이터 쓰고 싶은거니까..주소 따라가서 실제 메시지 꺼내ㅑ오기 
    path_file << "x,y\n";
  }

  ; // [2] ENU 변환 (offset 복원 + 모든 좌표변환은 coord_utils.cpp 안에서 수행,그 오프셋이나 고정값 다 coord_utils.cpp에 넣어둠..>메인 깔끔하게 하려고 근데 이게 나은 것인가.. 그냥 메인에 두는게 나은가..궁금)
  double x, y, z;
  egoToENU(*msg, lat0, lon0, h0, x, y, z);

  ; // [3] 0.1 m 간격 저장
  const bool first_point = std::isnan(last_x) || std::isnan(last_y);
  if (first_point || std::hypot(x - last_x, y - last_y) >= 0.1) {
    path_file << x << "," << y << "\n";  // 0.1 m 이상 움직였으면 새로운 웨이포인트 저장
    last_x = x; last_y = y;     // 이번 점을 마지막 저장점으로 새로 갱신하는거
  }
}

} // namespace path_recorder

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_recorder");
  ros::NodeHandle nh;

  auto sub = nh.subscribe("/Ego_topic", 50, path_recorder::egoCallback);

  ros::spin();
  path_recorder::path_file.close();
  return 0;
}

; // double은 자료형(변수 타입) 이고, 변수는 데이터를 저장하는 공간(소수점 숫자 저장 가능,15자리근처까지)
; // bool은 true/false 값을 가지는 자료형
; //자료형이란 이 변수에 어떤 종류의 데이터가 저장될 수 있는지를 정의하는 것
; // std::ofstream은 파일에 데이터를 쓰기 위한 스트림 객체
; // std::isnan은 숫자가 아닌지(NaN) 확인하는 함수
; // std::hypot는 피타고라스 정리를 이용해 두 점 사이의 거리를 계산하는 함수
;
// ; //네임 스페이스 한 이유
// ; 여러 소스 코드를 만들다보니까 콜백함수이름이라든지 노드이름,함수이름 이런거 겹쳐도 되는건가 궁금해짐..
// ; 예를 들면 좌표변환 함수에서 쓰는 에고콜백 이름을 그대로 여기 코드에도 갖고와도 되는지 궁금..
// ; 음 각각의 노드를 실행하면 (실행파일 각각 따로) 이름 같아도 같이 실행만 안하면 문제없긴함
// ; 그치만 같이 실행할 일(같이 실행시 재정의 에러 발생으로 충돌함)이 있을 수도 있으니까 네임스페이스로 분리하는게 좋음
// ; 이네임스페이스 도입해서 해결->네임스페이스로 분리하면 문제 없음

// ; extern : 다른 파일에 정의된 변수를 이 파일에서 사용하겠다는 의미 -> 여기서 또 쓰면 코드 넘 길어져서 이거 써봄..
// ; include는 헤더파일 가져오는거고 extern은 cpp파일에 있는 변수 가져오는거
// ; include는 떠먹여주는 느낌이고 extern은 선언만 하고 정의는 니가 알아서 경로 찾아라 느낌..

// ;  const bool first_point = std::isnan(last_x) || std::isnan(last_y);
; // 이번 콜백에서 "첫 웨이포인트인가?"를 판단하는 플래그.
; //   - std::isnan(v): v가 NaN(아직 값이 없음/정의되지 않음)이면 true.
; //   - 시작 직후 last_x, last_y는 NaN으로 초기화했으므로 첫 콜백에선 true가 됨.
; //   - 'const'를 붙인 이유: 이 값은 이번 콜백 실행 중 바뀌면 안 되는 '결과 값'이므로
; //     실수로 수정되는 걸 막음(불변 보장)

// ; if (first_point || std::hypot(x - last_x, y - last_y) >= 0.1) {
; //  저장 조건: (A) 첫 점이거나, (B) 이전 저장점에서 0.1m(10cm) 이상 떨어졌을 때만.
; //   - std::hypot(dx, dy): √(dx^2 + dy^2) — 2D 유클리드 거리.
; //     직접 sqrt(dx*dx + dy*dy)보다 overflow/underflow에 강하고 수치적으로 안정적.
; //   - ENU 좌표는 단위가 "미터"
; //   - 'first_point'가 true면 첫 점은 무조건 한 번 저장(시작점 보장)

// //| 표현                  | 의미          | 읽는 방법                    |
// | ------------------- | ----------- | ------------------------ |
// | `msg`               | 포인터 (주소)    | “집 주소”                   |
// | `*msg`              | 객체 (실제 데이터) | “집 자체”                   |
// | `(*msg).position.x` | 객체 안의 값 접근  | “주소 따라가서 집 들어가고 → 물건 찾기” |
// | `msg->position.x`   | 포인터 지름길     | “주소만으로 바로 집 물건 찾기”       |
// .

// 해야할 거 --> coord_utils.cpp 수정 해야함 + 기준점 추가 .. main2.cpp에서 정의되어잇던 오프셋값이나 고정값들 coord_utils.cpp로 옮기기