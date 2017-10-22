#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // TODO: Initialize proportional error
  // TODO: Initialize differential error
  // TODO: Initialize integral error
}

void PID::UpdateError(double cte) {
  // TODO: Update p_error
  // TODO: Update i_error
  // TODO: Update d_error
}

double PID::TotalError() {
  // TODO: Calculate total PID error
  // TODO: Return total PID error
}
