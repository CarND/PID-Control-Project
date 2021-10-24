#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  // void Init(double Kp_, double Ki_, double Kd_);
  void Init(std::vector<double> params, std::vector<double> dparams);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void TwiddleOptimizeCoef(double cte, double tol);

//  private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  std::vector<double> p; // use in place of Kp, Ki, Kd

  /**
   * twiddle PID-modification Coefficients
   */ 
  std::vector<double> dp; // use in place of dKp, dKi, dKd

  // loop count for twidder
  int it;
  bool is_initialized;

};

#endif  // PID_H