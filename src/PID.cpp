#include "PID.h"
#include <iostream>
#include <chrono>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Initialize proportional error
  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  cte_sum_ = 0.0;
  cte_previous_ = 0.0;
  t_previous_ = 0.0;
}

void PID::UpdateError(double cte) {
  // Calculate dt
  long long t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  double dt = (t - t_previous_);
  t_previous_ = t;
  std::cout << dt / 1000.0 << std::endl;
  
  // TODO: Remove the next line and tune the pid variables
  dt = 1.0;
  
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
