#include "coord_utils.hpp"
#include <cmath>
using namespace GeographicLib;

// ---- UTM → WGS84 ----
void utmToWgs84(double easting, double northing, int zone, bool northp,
                double& lat, double& lon)
{
    double gamma, k;
    UTMUPS::Reverse(zone, northp, easting, northing, lat, lon, gamma, k);
}

// ---- WGS84 → ECEF ----
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z)
{
    constexpr double a = 6378137.0;           // WGS84 반경
    constexpr double f = 1 / 298.257223563;   // WGS84 편평률
    constexpr double e2 = f * (2 - f);

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

    x = (N + h) * cos(lat_rad) * cos(lon_rad);
    y = (N + h) * cos(lat_rad) * sin(lon_rad);
    z = (N * (1 - e2) + h) * sin(lat_rad);
}

// ---- ECEF → ENU ----
void ecefToENU(double x, double y, double z,
               double lat0, double lon0, double h0,
               double& east, double& north, double& up)
{
    // 기준점 WGS84 → ECEF
    double x0, y0, z0;
    wgs84ToECEF(lat0, lon0, h0, x0, y0, z0);

    // 차이
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    double lat_rad = lat0 * M_PI / 180.0;
    double lon_rad = lon0 * M_PI / 180.0;

    // 변환 행렬
    double t[3][3] = {
        {-sin(lon_rad),             cos(lon_rad),              0},
        {-sin(lat_rad)*cos(lon_rad), -sin(lat_rad)*sin(lon_rad), cos(lat_rad)},
        { cos(lat_rad)*cos(lon_rad),  cos(lat_rad)*sin(lon_rad), sin(lat_rad)}
    };

    east  = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz;
    north = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz;
    up    = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz;
}
