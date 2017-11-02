#ifndef SISO_BLOCKS_H
#define SISO_BLOCKS_H

/*
  This file contains a set of Single-Input Single-Output (SISO) blocks used for
  the simulation and control of systems.
*/

/* Simulates a Single-Input Single-Output block */
class SISOBlock
{
public:
  SISOBlock(double inital_output = 0)
    : last_output(inital_output) {};

  /* Updates the output based on the input and time */
  double Update(double input, double timestep) {
    last_output = InternalUpdate(input, timestep);
    return last_output;
  }

  /* Returns last computed output */
  double Output() const {
    return last_output;
  }
protected:
  virtual double InternalUpdate(double input, double timestep) = 0;
private:
  double last_output;
};

/* Simulates a First-Order response of the form a(dy/dt) + by = x */
class FirstOrderResponseBlock : public SISOBlock
{
public:
  FirstOrderResponseBlock(double a, double b)
    : a(a), b(b) {};
protected:
  virtual double InternalUpdate(double input, double timestep) {
    /* Get last output */
    double y = Output();

    /* Calculate time derivatives from response formula */
    double ydot = (1 / a) * input - (b / a) * y;

    /* Apply Euler integration */
    y = (1 - (timestep*b) / a) * y + (timestep / a)*input;

    return y;
  }
private:
  double a, b;
};

#endif

