#include "PID.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>

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

void PID::Twiddle(double tolerance) {
  int size = 3;
  vector<double> dp(size, 1.0);
  Kp_ = 0.0;
  Ki_ = 0.0;
  Kd_ = 0.0;
  double best_err = TotalError();
  double err;
  double * K_ptr;
  
  while (accumulate(dp.begin(), dp.end(), 0.0) > tolerance) {
    for (int ix = 0; ix < size; ++ix) {
      // Pick the coefficient to adjust on each itteration
      if (ix == 0) { K_ptr = &Kp_; }
      else if (ix == 1) { K_ptr = &Ki_; }
      else { K_ptr = &Kd_; }
      
      *K_ptr += dp[ix];
      err = TotalError();

      if (err < best_err) {
        best_err = err;
        dp[ix] *= 1.1;
      } else {
        *K_ptr -= 2 * dp[ix];
        err = TotalError();

        if (err < best_err) {
          best_err = err;
          dp[ix] *= 1.1;
        } else {
          *K_ptr += dp[ix];
          dp[ix] *= 0.9;
        } // End inner if/else
        
      } // End outer if/else
    } // End for
  } // End while
  
  // Report the findings
  cout  << "Twiddler says: Kp = " << Kp_ << " Ki = " << Kd_ << " Kd = " << Kd_
        << " -> Best error = " << best_err << endl;
}
