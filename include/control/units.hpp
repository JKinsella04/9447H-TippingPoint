#pragma once
#include "globals.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QSpeed.hpp"

/*
 * The unit library we uses allows us to check our math during compile time
 * compile time using dimensional analysis. This file includes expansion
 * to extra units such as curvature that we use within our code.
 */
namespace okapi{
// Physical quantity types
QUANTITY_TYPE(0, -1, 0, 1, QCurvature); 

// Predegined Number unit
constexpr Number pct = number / 100;

// Predefined Length Units
constexpr QLength tile = 2 * foot;
constexpr QLength court = 12 * foot;
constexpr QLength revolution = 0.259207 * meter; // Diameter
constexpr QLength tick = revolution / 300; // 1800 red, 900 green, 300 blue.

// Predefined Angle Units
constexpr QAngle rotationTick = degree * 360 / 4096; 

// Predefined Curvature Unit
constexpr QCurvature radpm = radian / meter;

// Predefined Speed Unit
constexpr QSpeed ftps = foot / second;
constexpr QSpeed tps = tick / second;
constexpr QSpeed inps = inch / second;

// Predefined Acceleration Unit
constexpr QAcceleration ftps2 = ftps / second;
constexpr QAcceleration tps2 = tps / second;
constexpr QAcceleration inps2 = inps / second;

// Predefined Angular Acceleration Unit
constexpr QAngularAcceleration radps2 = radps / second;

// Predefined Jerk Unit
constexpr QJerk mps3 = mps2 / second;
constexpr QJerk ftps3 = ftps2 / second;

// Predefined Angular Jerk Unit
constexpr QAngularJerk radps3 = radps2 / second;

inline namespace literals{
// number unit literals
constexpr Number operator"" _pct(long double x) { return static_cast<double>(x)*pct; }
constexpr Number operator"" _pct(unsigned long long int x) { return static_cast<double>(x)*pct; }

// Length Unit Literals
constexpr QLength operator"" _tile(long double x) { return static_cast<double>(x)*tile; }
constexpr QLength operator"" _tile(unsigned long long int x) { return static_cast<double>(x)*tile; }
constexpr QLength operator"" _court(long double x) { return static_cast<double>(x)*court; }
constexpr QLength operator"" _court(unsigned long long int x) { return static_cast<double>(x)*court; }

constexpr QLength operator"" _rev(long double x) { return static_cast<double>(x)*revolution; }
constexpr QLength operator"" _rev(unsigned long long int x) { return static_cast<double>(x)*revolution; }
constexpr QLength operator"" _tick(long double x) { return static_cast<double>(x)*tick; }
constexpr QLength operator"" _tick(unsigned long long int x) { return static_cast<double>(x)*tick; }

// Curvature Unit Literals
constexpr QCurvature operator"" _radpm(long double x) { return static_cast<double>(x)*radpm; }
constexpr QCurvature operator"" _radpm(unsigned long long int x) { return static_cast<double>(x)*radpm; }

// Predefined Speed Unit
constexpr QSpeed operator"" _ftps(long double x) { return static_cast<double>(x)*ftps; }
constexpr QSpeed operator"" _ftps(unsigned long long int x) { return static_cast<double>(x)*ftps; }

constexpr QSpeed operator"" _tps(long double x) { return static_cast<double>(x)*tps; }
constexpr QSpeed operator"" _tps(unsigned long long int x) { return static_cast<double>(x)*tps; }

constexpr QSpeed operator"" _inps(long double x) { return static_cast<double>(x)*inps; }
constexpr QSpeed operator"" _inps(unsigned long long int x) { return static_cast<double>(x)*inps; }

// Angular Speed literals
constexpr QAngularSpeed operator"" _radps(long double x) { return static_cast<double>(x)*radps; }
constexpr QAngularSpeed operator"" _radps(unsigned long long int x) { return static_cast<double>(x)*radps; }

// Predefined Acceleration Unit
constexpr QAcceleration operator"" _ftps2(long double x) { return static_cast<double>(x)*ftps2; }
constexpr QAcceleration operator"" _ftps2(unsigned long long int x) { return static_cast<double>(x)*ftps2; }

constexpr QAcceleration operator"" _tps2(long double x) { return static_cast<double>(x)*tps2; }
constexpr QAcceleration operator"" _tps2(unsigned long long int x) { return static_cast<double>(x)*tps2; }

constexpr QAcceleration operator"" _inps2(long double x) { return static_cast<double>(x)*inps2; }
constexpr QAcceleration operator"" _inps2(unsigned long long int x) { return static_cast<double>(x)*inps2; }

// Angular Acceleration literals
constexpr QAngularAcceleration operator"" _radps2(long double x) { return static_cast<double>(x)*radps2; }
constexpr QAngularAcceleration operator"" _radps2(unsigned long long int x) { return static_cast<double>(x)*radps2; }

// Jerk Literals
constexpr QJerk operator"" _mps3(long double x) { return static_cast<double>(x)*mps3; }
constexpr QJerk operator"" _mps3(unsigned long long int x) { return static_cast<double>(x)*mps3; }
constexpr QJerk operator"" _ftps3(long double x) { return static_cast<double>(x)*ftps3; }
constexpr QJerk operator"" _ftps3(unsigned long long int x) { return static_cast<double>(x)*ftps3; }

// Angular Jerk Literals
constexpr QAngularJerk operator"" _radps3(long double x) { return static_cast<double>(x)*radps3; }
constexpr QAngularJerk operator"" _radps3(unsigned long long int x) { return static_cast<double>(x)*radps3; }

} // namespace literals
} // namespace okapi