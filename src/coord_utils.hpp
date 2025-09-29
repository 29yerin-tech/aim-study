#ifndef COORD_UTILS_HPP
#define COORD_UTILS_HPP

// ───── WGS84 타원체 상수 ─────
extern const double a;   // 장반경
extern const double f;   // 편평률
extern const double e2;  // 이심률 제곱
extern const double k0;  // UTM 스케일 계수

// ───── 함수 선언 ─────

// WGS84 → ECEF
void wgs84ToECEF(double lat, double lon, double h,
                 double &x, double &y, double &z);

// ECEF → ENU
void ecefToENU(double x, double y, double z,
               double x0, double y0, double z0,
               double lat0, double lon0,
               double &east, double &north, double &up);

// UTM → WGS84
void utmToWgs84(double easting, double northing, int zone, bool northHemisphere,
                double &lat, double &lon);

#endif // COORD_UTILS_HPP
