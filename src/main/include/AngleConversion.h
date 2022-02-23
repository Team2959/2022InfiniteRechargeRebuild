#pragma once

static constexpr double PI{ 3.14159265359 };
static constexpr double DegreesToRadiansFactor{ PI / 180.0 };

static constexpr double DegreesToRadians(double degrees) { return degrees * DegreesToRadiansFactor; }
static constexpr double RadiansToDegrees(double radians) { return radians / DegreesToRadiansFactor; }
