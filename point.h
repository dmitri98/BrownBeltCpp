#pragma once

#include <cmath>

const double PI = 3.1415926535;
const double R = 6371000.0;

struct Point {
	Point(double latitude, double longitude);
	Point(const Point& other);

	static double GetRadians(double degrees);

	const double latitude_;
	const double longitude_;
};

double GetDistance(const Point& p1, const Point& p2);
