#ifndef DP_CAR_H
#define DP_CAR_H


#include <vector>
#include <math.h>
#include <sstream>
//#include <iostream>
//#include <iomanip>


using namespace std;

class Vehicle {
public:
  int id;
  double s;
  double s_dot;
  double s_ddot;
  double d;
  double d_dot;
  double d_ddot;

  Vehicle(const int identifier);


  void updatePos(double s, double d);
  void updateVelocity(double s_dot, double d_dot);

  void followTrajectory(const vector<double>& path_s, const vector<double>& path_d, size_t steps);

  Car stateAt(double dt) const;

  static size_t getLane() const;

  bool crashWith(const Car& other) const;

  friend std::ostream& operator<<(std::ostream& stream, const Car& matrix);

  void display() const;

  double getLaneD() const;
};

#endif
