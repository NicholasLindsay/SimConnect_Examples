#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "common\siso_blocks.h"

/* 
  Provides a simple and output clamped PID controller for use in simulations.
*/

/* Generic PID controller */
class PIDController : public SISOBlock
{
public:
  PIDController(double p_coeff, double d_coeff, double i_coeff)
    : p_coeff(p_coeff), d_coeff(d_coeff), i_coeff(i_coeff), last_error(0), error_integral(0) {}

  void SetPCoefficient(double val) {
    p_coeff = val;
  }
  void SetDCoefficient(double val) {
    d_coeff = val;
  }
  void SetICoefficient(double val) {
    i_coeff = val;
  }

  double GetPCoefficient() const {
    return p_coeff;
  }
  double GetDCoefficient() const {
    return d_coeff;
  }
  double GetICoefficient() const {
    return i_coeff;
  }

protected:
  /* Internal PID calculation */
  virtual double InternalUpdate(double new_error, double timestep) override {
    error_integral += new_error * timestep;
    double error_diff = (new_error - last_error) / timestep;

    double p = p_coeff * new_error;
    double i = i_coeff * error_integral;
    double d = d_coeff * error_diff;

    last_error = new_error;

    return p + i + d;
  }

private:
  double p_coeff, d_coeff, i_coeff; 

  double last_error;
  double error_integral;
};

/* PID controller, with clamping */
class ClampedPIDController : public PIDController
{
public:
  ClampedPIDController(double p_coeff, double d_coeff, double i_coeff, double lowClamp, double highClamp)
    : PIDController(p_coeff, d_coeff, i_coeff), clampLow(lowClamp), clampHigh(highClamp) {};

  /* Set the clamping limits */
  void SetClampingLimits(double lower, double higher) {
    clampLow = lower;
    clampHigh = higher;
  }

  /* Return lower clamping limit */
  double GetClampLowLimit() const {
    return clampLow;
  }

  /* Return higher clamping limit */
  double GetClampHighLimit() const {
    return clampHigh;
  }

protected:
  /* Update and clamp output */
  virtual double InternalUpdate(double new_error, double timestep) override {
    double res = PIDController::InternalUpdate(new_error, timestep);
    if (res > clampHigh) {
      res = clampHigh;
    }
    else if (res < clampLow) {
      res = clampLow;
    }
    return res;
  }

private:
  double clampLow, clampHigh;
};

#endif
