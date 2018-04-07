#ifndef PID_H
#define PID_H
#include <iostream>
#include "Eigen/Dense"

using Eigen::Vector3d;

class PID {
public:
  /*
  * Errors
  */
  Vector3d errors_;
  /*
  * Coefficients
  */ 
  Vector3d K_;

  // steering value
  double steer_value;

  // to store cte_
  double cte_;

  // for accumulate error
  int counter_;
  int epoch_length_;
  Vector3d dp_;
  double sum_err_;
  double best_err_;
  int param_index_;
  int skip_steps;

  bool training;
  bool isDescending;

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

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Check if one epoch is finished
  */
  bool IsEpochEnd();

  /* 
  * Update 
  */
  void UpdateK(int index, double delta);

};

#endif /* PID_H */
