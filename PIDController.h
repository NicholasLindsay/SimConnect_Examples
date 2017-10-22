#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

/* Generic PID controller */
class PIDController
{
public:
  PIDController(double p_coeff, double d_coeff, double i_coeff)
    : p_coeff(p_coeff), d_coeff(d_coeff), i_coeff(i_coeff), last_error(0), error_integral(0), last_output(0) {}

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
  
  /* 
     Updates the PID controller with a new error and timestep and recalculates
     the output accordingling.
  */
  double Update(double new_error, double timestep) {
    error_integral += new_error * timestep;
    double error_diff = (new_error - last_error) / timestep;

    double p = p_coeff * new_error;
    double i = i_coeff * error_integral;
    double d = d_coeff * error_diff;

    last_output = p + i + d;

    last_error = new_error;

    return last_output;
  }

  /*
    Returns the current output calculated after the last update;
  */
  double Output() const {
    return last_output;
  }
private:
  double p_coeff, d_coeff, i_coeff; 

  double last_error;
  double error_integral;
  double last_output;
};

#endif
