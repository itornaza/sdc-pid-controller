#include "PID.h"
#include <iostream>
#include <chrono>

using namespace std;
using namespace chrono;

const double MSEC_2_SEC = 1000.0;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Timestamp
  t_previous_ = 0.0;
  
  // Errors
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  
  // Coefficients
  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
}

void PID::UpdateError(double cte) {
  // Calculate dt
  long long t = duration_cast<milliseconds>
                (system_clock::now().time_since_epoch()).count();
  double dt = (t - t_previous_) / MSEC_2_SEC;
  t_previous_ = t;
  
  // Update errors
  d_error_ = (cte - p_error_) / dt;
  p_error_ = cte;
  i_error_ += cte;
}

double PID::TotalError() {
  return -(Kp_ * p_error_) -(Kd_ * d_error_) - (Ki_ * i_error_);
}
