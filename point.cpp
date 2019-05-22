#include "point.h"

Point::Point(double latitude, double longitude)
             : latitude_(GetRadians(latitude))
             , longitude_(GetRadians(longitude))
{}

Point::Point(const Point& other) = default;

double Point::GetRadians(double degrees) {
	return degrees * PI / 180.0;
}

double GetDistance(const Point& p1, const Point& p2) {
	double res = sin(p1.latitude_) * sin(p2.latitude_);
	res += cos(p1.latitude_) * cos(p2.latitude_) * cos(p1.longitude_ - p2.longitude_);
	return R * acos(res);
}
