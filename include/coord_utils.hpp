#pragma once
#include <GeographicLib/UTMUPS.hpp>

// ---- 좌표 변환 함수 선언 ----

// UTM → WGS84 (GeographicLib 사용)
void utmToWgs84(double easting, double northing, int zone, bool northp,
                double& lat, double& lon);

// WGS84 → ECEF 
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z);

// ECEF → ENU 
void ecefToENU(double x, double y, double z,
               double lat0, double lon0, double h0,
               double& east, double& north, double& up);
