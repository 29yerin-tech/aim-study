#pragma once
#include <morai_msgs/EgoVehicleStatus.h>

// WGS84 → ECEF
void wgs84ToECEF(double lat, double lon, double h,
                 double &x, double &y, double &z);

// ECEF → ENU
void ecefToENU(double x, double y, double z,
               double x0, double y0, double z0,
               double lat0, double lon0,
               double &east, double &north, double &up);

// UTM → WGS84
void utmToWgs84(double easting, double northing,
                int zone, bool northHemisphere,
                double &lat, double &lon);

// Origin 설정
void setOriginFromEgo(const morai_msgs::EgoVehicleStatus& msg,
                      double &lat0, double &lon0, double &h0);

// Ego → ENU 변환
void egoToENU(const morai_msgs::EgoVehicleStatus& msg,
              double lat0, double lon0, double h0,
              double &enu_x, double &enu_y, double &enu_z);

