#include "coord_utils.hpp"
#include <cmath>

// ───── WGS84 상수 정의 ─────
const double a = 6378137.0;              // 장반경 (m)
const double f = 1 / 298.257223563;      // 편평률
const double e2 = 2*f - f*f;             // 이심률 제곱
const double k0 = 0.9996;                // UTM scale factor

// ───── WGS84 → ECEF ─────
void wgs84ToECEF(double lat, double lon, double h,
                 double &x, double &y, double &z) {
    lat = lat * M_PI / 180.0;
    lon = lon * M_PI / 180.0;
    double N = a / sqrt(1 - e2 * pow(sin(lat), 2));
    x = (N + h) * cos(lat) * cos(lon);
    y = (N + h) * cos(lat) * sin(lon);
    z = (N * (1 - e2) + h) * sin(lat);
}

// ───── ECEF → ENU ─────
void ecefToENU(double x, double y, double z,
               double x0, double y0, double z0,
               double lat0, double lon0,
               double &east, double &north, double &up) {
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    lat0 = lat0 * M_PI / 180.0;
    lon0 = lon0 * M_PI / 180.0;

    east  = -sin(lon0) * dx + cos(lon0) * dy;
    north = -sin(lat0) * cos(lon0) * dx - sin(lat0) * sin(lon0) * dy + cos(lat0) * dz;
    up    =  cos(lat0) * cos(lon0) * dx + cos(lat0) * sin(lon0) * dy + sin(lat0) * dz;
}

// ───── UTM → WGS84 ─────
void utmToWgs84(double easting, double northing, int zone, bool northHemisphere,
                double &lat, double &lon) {
    double eccPrimeSquared = e2 / (1 - e2);
    double e1 = (1 - sqrt(1 - e2)) / (1 + sqrt(1 - e2));

    double x = easting - 500000.0;
    double y = northing;
    if (!northHemisphere) y -= 10000000.0;

    double M = y / k0;
    double mu = M / (a * (1 - e2/4 - 3*e2*e2/64 - 5*e2*e2*e2/256));

    double phi1Rad = mu
        + (3*e1/2 - 27*pow(e1,3)/32) * sin(2*mu)
        + (21*e1*e1/16 - 55*pow(e1,4)/32) * sin(4*mu)
        + (151*pow(e1,3)/96) * sin(6*mu);

    double N1 = a / sqrt(1 - e2 * pow(sin(phi1Rad), 2));
    double T1 = pow(tan(phi1Rad), 2);
    double C1 = eccPrimeSquared * pow(cos(phi1Rad), 2);
    double R1 = a * (1 - e2) / pow(1 - e2 * pow(sin(phi1Rad),2), 1.5);
    double D = x / (N1 * k0);

    lat = phi1Rad - (N1 * tan(phi1Rad) / R1) *
          (D*D/2 - (5 + 3*T1 + 10*C1 - 4*C1*C1 - 9*eccPrimeSquared) * pow(D,4)/24
          + (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*eccPrimeSquared - 3*C1*C1) * pow(D,6)/720);
    lat = lat * 180.0 / M_PI;

    lon = (D - (1 + 2*T1 + C1) * pow(D,3)/6
          + (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*eccPrimeSquared + 24*T1*T1) * pow(D,5)/120) / cos(phi1Rad);
    lon = (zone > 0 ? (zone * 6 - 183.0) : 3.0) + lon * 180.0 / M_PI;
}
