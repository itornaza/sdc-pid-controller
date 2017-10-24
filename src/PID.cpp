#include "PID.h"
#include <iostream>
#include <chrono>

using namespace std;
using namespace chrono;

const double MSEC_2_SEC = 1000.0;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Errors
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  
  // Helper variables
  cte_sum_ = 0.0;
  cte_previous_ = 0.0;
  t_previous_ = 0.0;
  
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
  
  // CTE calculations
  double diff_cte = cte - cte_previous_;
  cte_sum_ += cte;
  cte_previous_ = cte;
  
  // Update errors
  p_error_ = cte;
  d_error_ = diff_cte / dt;
  i_error_ = cte_sum_;
}

double PID::TotalError() {
  return -(Kp_ * p_error_) -(Kd_ * d_error_) - (Ki_ * i_error_);
}
