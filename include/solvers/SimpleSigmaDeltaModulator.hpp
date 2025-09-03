#ifndef SIGMA_DELTA_MODULATORS_HPP
#define SIGMA_DELTA_MODULATORS_HPP

#include <array>
#include <Eigen/Dense>
#include <iostream>

template <unsigned int NUMBER_THRUSTERS>
class SimpleSigmaDeltaModulators
{
public:
  /**
   * @brief Constructs a sigma delta modulator array that modulates a continuous force onto binary thrusters actuation
   * @param k Gain for the integrators
   * @param eps Threshold for the integrators to actuate thrusters
   */
  SimpleSigmaDeltaModulators(double k, double eps) : K(k), EPS(eps)
  {
    this->t_last = 0.0;
    reset();
  }

  /**
   * @brief Converts a continuous force at a given timestamp to a binary output command for the thrusters
   * @param force Array of forces to convert to binary outputs
   * @param t The timestamp of the sampled force values
   * @return Array of modulated outputs for the actuators: true = opened, false = closed
   */
  Eigen::Vector<double, NUMBER_THRUSTERS> modulate_continuous_force(const Eigen::Vector<double, NUMBER_THRUSTERS> &input,
                                                                    const double dt)
  {

    for (size_t i = 0; i < NUMBER_THRUSTERS; i++)
    {
      // Integrate the current error weighted by the system gain k
      this->integrator_value(i) += dt * this->K * (input(i) - this->current_output(i));

      // Update the current output
      this->current_output(i) = this->integrator_value(i) > this->EPS;
    }

    return this->current_output;
  }

  /**
   * @brief Resets the modulator to its default state, i.e. zeroes the output, integrated error and last sample
   * timestamp
   */
  void reset()
  {
    std::cout << "Resetting Sigma Delta Modulator" << std::endl;
    this->t_last = 0.0;
    for (size_t i = 0; i < NUMBER_THRUSTERS; i++)
    {
      this->integrator_value.setZero();
      this->current_output.setZero();
    }
  }

private:
  const double K, EPS;                                        // Gain in the feedforward path and threshold to trigger pulse
  double t_last;                                              // Last sampled values timestamp
  Eigen::Vector<double, NUMBER_THRUSTERS> integrator_value{}; // The integrator value the error is accumulated at
  Eigen::Vector<double, NUMBER_THRUSTERS> current_output{};     // Current output of the modulators
};

#endif // SIGMA_DELTA_MODULATORS_HPP