#pragma once

#include <array>
#include <Eigen/Dense>
#include <iostream>

template <unsigned int NUMBER_THRUSTERS>
class LimetingSigmaDeltaModulators
{
public:
  enum THRUSTER_STATE
  {
    ON,
    COOL_DOWN,
    OFF
  };

  /**
   * @brief Constructs a sigma delta modulator array that modulates a continuous force onto binary thrusters actuation
   * @param k Gain for the integrators
   * @param eps Threshold for the integrators to actuate thrusters
   */
  LimetingSigmaDeltaModulators(double k, double eps, double min_time_on, double max_time_on, double min_time_off) : K(k), EPS(eps), min_time_on_(min_time_on), max_time_on_(max_time_on), min_time_off_(min_time_off)
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
      switch (thruster_states_[i])
      {
      case ON:
        if (thruster_times_[i] > max_time_on_ || (thruster_times_[i] >= min_time_on_ && this->integrator_value(i) <= this->EPS))
        {
          // Goto cool down
          thruster_times_[i] = 0;
          this->current_output(i) = 0;
          thruster_states_[i] = COOL_DOWN;
        }
        break;
      case OFF:
        // Check if turn on
        if (this->integrator_value(i) > this->EPS)
        {
          thruster_times_[i] = 0;
          this->current_output(i) = 1;
          thruster_states_[i] = ON;
        }
        break;
      case COOL_DOWN:
        // Check if done cooling down, and maybe also directly going to fire
        if (thruster_times_[i] >= min_time_off_ && this->integrator_value(i) > this->EPS)
        {
          thruster_times_[i] = 0;
          this->current_output(i) = 1;
          thruster_states_[i] = ON;
        }
        else if (thruster_times_[i] >= min_time_off_)
        {
          thruster_times_[i] = 0;
          this->current_output(i) = 0;
          thruster_states_[i] = OFF;
        }
        break;
      }
      // Integrate the current error weighted by the system gain k
      this->integrator_value(i) += dt * this->K * (input(i) - this->current_output(i));
      thruster_times_[i] += dt;
    }

    return this->current_output;
  }

  template<unsigned int FIRING_HORIZON>
  void GetFutureFirings(Eigen::Ref<Eigen::Matrix<double, NUMBER_THRUSTERS, FIRING_HORIZON>> firings, Eigen::Ref<Eigen::Matrix<double, NUMBER_THRUSTERS, FIRING_HORIZON>> limits, const double predict_dt) const
  {
    firings.setZero();
    for (size_t i = 0; i < NUMBER_THRUSTERS; i++)
    {
      double integrator_value = this->integrator_value(i);
      THRUSTER_STATE thruster_state = this->thruster_states_[i];
      double thruster_time = this->thruster_times_[i];
      for (size_t j = 0; j < FIRING_HORIZON; j++)
      {
        switch (thruster_state)
        {
        case ON:
          if (thruster_time > max_time_on_ || (thruster_time >= min_time_on_ && integrator_value <= this->EPS))
          {
            // Goto cool down
            thruster_time = 0;
            firings(i, j) = 0;
            limits(i, j) = 0;
            thruster_state = COOL_DOWN;
          }
          else
          {
            firings(i, j) = 1;
            limits(i, j) = 1;
          }
          break;
        case OFF:
          // Check if turn on
          if (integrator_value > this->EPS)
          {
            thruster_time = 0;
            firings(i, j) = 1;
            thruster_state = ON;
            limits(i, j) = 1;
          }
          else
          {
            firings(i, j) = 0;
            limits(i, j) = 1;
          }
          break;
        case COOL_DOWN:
          // Check if done cooling down, and maybe also directly going to fire
          if (thruster_time >= min_time_off_ && integrator_value > this->EPS)
          {
            thruster_time = 0;
            firings(i, j) = 1;
            limits(i, j) = 1;
            thruster_state = ON;
          }
          else if (thruster_time >= min_time_off_)
          {
            thruster_time = 0;
            firings(i, j) = 0;
            limits(i, j) = 1;
            thruster_state = OFF;
          }
          else
          {
            firings(i, j) = 0;
            limits(i, j) = 0;
          }
          break;
        }
        integrator_value += predict_dt * this->K * (0 - firings(i,j));
        thruster_time += predict_dt;
      }
    }
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
      this->thruster_states_.fill(OFF);
      this->thruster_times_.fill(0.0);
    }
  }

private:
  const double min_time_on_, max_time_on_, min_time_off_;
  const double K, EPS;                                      // Gain in the feedforward path and threshold to trigger pulse
  double t_last;                                            // Last sampled values timestamp
  Eigen::Vector<double, NUMBER_THRUSTERS> integrator_value; // The integrator value the error is accumulated at
  Eigen::Vector<double, NUMBER_THRUSTERS> current_output;   // Current output of the modulators
  std::array<double, NUMBER_THRUSTERS> thruster_times_;
  std::array<THRUSTER_STATE, NUMBER_THRUSTERS> thruster_states_;
};
