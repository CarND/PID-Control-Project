#include "PID.h"
#include <cmath>  
#include <vector>
#include <iostream>

/**
 * Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

// void PID::Init(double Kp_, double Ki_, double Kd_) {
void PID::Init(std::vector<double> params, std::vector<double> dparams) {

  /**
   * Initialize PID coefficients and errors
   */
  p = params;
  dp = dparams;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  it = 0;
  is_initialized = false;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  double cte_diff = cte - p_error;
  p_error = cte;
  i_error += cte ;
  d_error = cte_diff;
  
  // std::cout << "UpdateError total error: " << TotalError()<< std::endl;


}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return (-p[0]*p_error - p[1]*i_error - p[2]*d_error);  
}


void PID::TwiddleOptimizeCoef(double cte, double tol) {
    // Twiddle algorithm translated to cpp from Sebastian Thrun's python code.
    // Not used here since manual tuning used.  Furthermore, it is not recommended
    // to use this algorithm in live unrepeatable environment since twiddle 
    // requires repititious evaluations but with different values to determine
    // how each variable contributes to changes in error.  Live environments 
    // change so that running same evaluation but with different param will
    // yield results that rely on changed environments so cannot determine 
    // whether changes attributable to new tweak or changed environment. 

    // first frame cte used to calc total error - set as best initially
    if (!is_initialized) {
      is_initialized = true;
      std::cout << "TwiddleOptimize initialization: " << is_initialized << " cte: " << cte 
                << std::endl;      
      return;
    }

    std::cout << "TwiddleOptimize is initialized: " << is_initialized << " cte: " << cte 
              << std::endl;

    std::cout << "Twiddle it: " << it 
              << std::endl;

    // subsequent frames optimization as long as total error above tolerance
    if (std::abs(TotalError()) > tol) 
    { 
      
      bool err_reduced;

      // check if error is less after change in params
      if (std::abs(cte) < std::abs(p_error)){
        err_reduced = true;
      } else {
        err_reduced = false;
      }

      std::cout << "p_error: " << p_error << " cte: " << cte << " err_reduced: " << err_reduced
                << std::endl;

      // first term
      if (it % 9 == 0) 
      {
        p[0] += dp[0]; // test new p value
        std::cout << "it % 9 == 0: " << (it % 9 == 0) << " p[0]: " << p[0] << " dp[0]: " << dp[0]
                << std::endl;

      } else if (it % 9 == 1) {
        if ( err_reduced ) 
        {
          dp[0] *= 1.1;
          it += 3; // skip to mod 4
          p[1] += dp[1]; // test new second term
          return; // to skip it++
        } 
        else 
        {
          p[0] -= 2 * dp[0];
        }
      } 
      else if (it % 9 == 2) 
      {
        if ( err_reduced ) 
        {
          dp[0] *= 1.1;
        } 
        else 
        {
          dp[0] *= 0.9;
        }        
      } // end first term
      // second term
      else if (it % 9 == 3) 
      {
        p[1] += p[1]; // test new p value
      } 
      else if (it % 9 == 4) 
      {
        if ( err_reduced ) 
        {
          dp[1] *= 1.1;
          it += 3; // skip to mod 7
          p[2] += dp[2]; // test new third term value
          return; // to skip it++
        } 
        else 
        {
          p[1] -= 2 * dp[1];
        }
      } 
      else if (it % 9 == 5) 
      {
        if ( err_reduced ) 
        {
          dp[1] *= 1.1;
        } 
        else 
        {
          dp[1] *= 0.9;
        }        
      } // end second term

      // third term
      else if (it % 9 == 6) 
      {
        p[2] += dp[2]; // test new p value
      } 
      else if (it % 9 == 7) 
      {
        if ( err_reduced ) 
        {
          dp[2] *= 1.1;
          it += 3; // skip to mod 1
          p[0] += dp[0]; // test new first term value (cycle back)
          return; // to skip it++
        } 
        else 
        {
          p[2] -= 2 * dp[2];
        }
      } 
      else if (it % 9 == 8) 
      {
        if ( err_reduced ) 
        {
          dp[2] *= 1.1;
        } 
        else 
        {
          dp[2] *= 0.9;
        }        
      } // end third term
      else
      {
        // nothing to do here
      }
      
      it++;

    }
    
}

