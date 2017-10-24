#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /**
   * Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;
  
  /**
   * Helper variables
   */
  double cte_sum_;
  double cte_previous_;
  long long t_previous_;
  
  /**
   * Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor
   */
  virtual ~PID();

  /**
   * Initialize PID
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error
   */
  double TotalError();
  
  /*
   * Optimise the PID coefficients using the twiddle algorithm
   */
  void Twiddle(double tolerance);
};

#endif /* PID_H */
