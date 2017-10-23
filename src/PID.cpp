#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Initialize proportional error
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
  this->cte_sum = 0.0;
  this->cte_previous = 0.0;
}

void PID::UpdateError(double cte) {
  // TODO: Calculate dt from sim
  double dt = 1.0;
  double diff_cte = cte - cte_previous;
  cte_sum += cte;
  cte_previous = cte;
  
  // Update errors
  p_error = cte;
  d_error = diff_cte / dt;
  i_error = cte_sum;
}

double PID::TotalError() {
  double steering = -(Kp * p_error) -(Kd * d_error) - (Ki * i_error);
  return  steering;
}
