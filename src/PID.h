#ifndef PID_H
#define PID_H

class PID {
public:
  const double MAX_STEER = 1;
  const double MIN_STEER = -1;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /**
   * Get current recommended steering angle
   * @return steering value [-1,1]
   */
  double steering();

};

#endif /* PID_H */
