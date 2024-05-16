/**
 * @file Solver.hpp
 * @brief Abstract class that represents any solver interface and provides a low-lever mpc solver interface
 *
 * @author Franek Stark
 * Contact: franek.stark@dfki.de
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include "System.hpp"

namespace mimpc {
/**
 * Solver return code definitions
 */
    enum SOLVER_RETURN {
        /**
         * Optimal solution was found (within time solver limit)
         */
        OPTIMAL = 1,
        /**
         * No solution was found (within time solver limit)
         */
        NO_SOLUTION = 0,
        /**
         * Suboptimal/feasible solution was found within time solver limit
         */
        TIME_LIMIT_WITH_SOLUTION = 2,
        /**
        * Solver was interrupted by user (i.e. by pressing CTRL+C while solver was running)
        */
        USER_INTERRUPT = -10
    };

/**
 * Types of how the system state and input error is measured
 */
    enum COST_TYPE {
        /**
         * Linear cost term representing the L1 Norm (absolute distance)
         */
        L1Linear = 0,
        /**
         * Quadratic cost term representing the L2 Norm (euclidean distance)
         */
        L2Quadratic = 1
    };

/**
 * Ways of how the system dynamics are discretized
 */
    enum INTEGRATION_SCHEME {
        /**
         * forward (explicit) euler discretization
         */
        FORWARD_EULER = 0,
        /**
         * backward (implicit) euler discretization
         */
        BACKWARD_EULER = 1
    };

/**
 * This defines abstract base class defines a wrapper around any mixed integer solver and defines a low level solver interface to be used within an mpc.
 * For a highlevel interface refer to MPC, which takes any Solver implementation as a template argument.
 *
 * The solver discretizes the time into time steps of same length.
 *
 * @tparam SystemType refers to the system dynamics and must be a class implementing System
 * @tparam N prediction horizon
 * @tparam min_steps_on number of discrete time steps that the binary inputs have to stay on
 * @tparam min_steps_off number of discrete time steps that the binary inputs have to stay off
 * @tparam max_steps_on maximum number of discrete time steps that the binary inputs can stay on
 * @tparam num_steps_solver_delay number of discrete time steps that are solver delay, referring to the time delay from the time point system is at a current state (which will be set by setState()) and the time point at which the optimal input returned by solve() is applied to the system.
 * This delay is compensated by predicting the systems state based on the inputs that will be applied during this delay time. This inputs are set via setNextInputs() and can be for example taken from the previous open loop prediction.
 * Obviously, the optimal input predicted by the solver and returned by solve() is the one at num_steps_solver_delay in the next open loop prediction, as the first refer to the predicted states during the delay. The correct index for the next input can be aquired using getStepsToCompensateControllerDelay().
 * @tparam integration_scheme discretization method
 */
    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme> requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    class Solver {
    public:
        using SYSTEM_TYPE = SystemType;
        static constexpr auto PRED_STEPS = N;
        static constexpr auto MIN_STEPS_ON = min_steps_on;
        static constexpr auto MIN_STEPS_OFF = min_steps_off;
        static constexpr auto MAX_STEPS_ON = max_steps_on;
        static constexpr auto NUM_STEPS_SOLVER_DELAY = num_steps_solver_delay;
        static constexpr auto INTEGRATION_SCHEME = integration_scheme;

        /**
         * Gives, based on the timing constraints the number of past discrete time steps that the optimization problem needs to know
         */
        static constexpr int history_depth = std::max(
                {0, max_steps_on - num_steps_solver_delay, min_steps_off - num_steps_solver_delay,
                 min_steps_on - num_steps_solver_delay}); //retunrs 0 or biggest of thoose
        virtual ~Solver() = default;

        /**
         * Adds or replaces the bounding box constraint to the input variable with given index to all discrete time steps.
         *
         * @param index index of the respective input variable in the input vector
         * @param lb lower bound
         * @param ub upper bound
         */
        virtual void addInputConstraintOnIndex(int index, double lb, double ub) = 0;

        /**
         * Adds or replaces the bounding box constraint to the state variable with given index to all discrete time steps.
         *
         * @param index index of the respective state variable in the state vector
         * @param lb lower bound
         * @param ub upper bound
         */
        virtual void addStateConstraintOnIndex(int index, double lb, double ub) = 0;

        /**
         * Adds or replaces the bounding box constraint to the input vector at given time step
         *
         * @param step time step 0<=step<=N
         * @param lb lower bound
         * @param ub upper bound
         */
        virtual void
        addInputConstraintOnStep(int step, const SystemType::InputVec &lb, const SystemType::InputVec &ub) = 0;

        /**
         * Adds or replaces the bounding box constraint to the state vector at given time step
         *
         * @param step time step 0<=step<=N
         * @param lb lower bound
         * @param ub upper bound
         */
        virtual void
        addStateConstraintOnStep(int step, const SystemType::StateVec &lb, const SystemType::StateVec &ub) = 0;

        /**
         * Sets the state error weights of the final predicted state
         *
         * @param final_weights weight vector where each vector element refers to the respective state vector element
         */
        virtual void setFinalWeights(const SystemType::StateVec &final_weights) = 0;

        /**
         * Sets the state error weights of all intermediate predicted states
         *
         * @param state_weights weight vector where each vector element refers to the respective state vector element
         */
        virtual void setStateWeights(const SystemType::StateVec &state_weights) = 0;

        /**
          * Sets the input weights of all intermediate predicted inputs
          *
          * @param state_weights weight vector where each vector element refers to the respective input vector element
          */
        virtual void setInputWeights(const SystemType::InputVec &state_weights) = 0;

        /**
         * Sets the error function type
         *
         * @param cost_type error function type
         */
        virtual void setCost(COST_TYPE cost_type) = 0;

        /**
         * Sets the state set point. The state error is then calculated by applying the error function (as defined by setCost) to the difference between the current or predicted state and the set point.
         * Each state error vector element is weighted by the state and final weights as specified by setStateWeights and setFinalWeights
         *
         * @param set_point the state set point
         */
        virtual void setSetPoint(const SystemType::StateVec &set_point) = 0;

        /**
         * Sets the current system state which is then used to do the optimization and prediction process.
         *
         * @param state system state
         */
        virtual void setState(const SystemType::StateVec &state) = 0;

        /**
         * Sets the input history of past inputs. This is necessary in order to respect the timing constraints.
         *
         * @param bin_input_hist input history matrix, where the first index refers to the input vector element and the second index to the history time step.
         * Thereby index 0 refers to the most past element, and index #history_depth refers to the current time step.
         */
        virtual void setInputHistory(
                const Eigen::Matrix<double, SystemType::NUM_INPUTS, history_depth> &bin_input_hist) = 0;

        /**
         * Sets the inputs that the will be applied to the system during the solver delay, in order to predicts the systems behaviour while the solver optimizes the next input
         *
         * @param next_inputs input sequence
         */
        virtual void
        setNextInputs(const Eigen::Matrix<double, SystemType::NUM_INPUTS, num_steps_solver_delay> &next_inputs) = 0;

        /**
         * Sets the maximum time that the solver will try to find the optimal solution when solve is called. If the maximum time is reached, the best solution found so far will be returned.
         * @param max_seconds
         */
        virtual void setSolverTimeLimit(double max_seconds) = 0;

        /**
         * Returns the index at which the next input calculated by solve() has to be taken, in order to correct respect the solver delay compensation. Most of the times this value should refer to #num_steps_solver_delay
         * @return num_steps_solver_delay
         */
        virtual unsigned int getStepsToCompensateControllerDelay() = 0;

        /**
         * Starts the solving process in order to find the optimal input and state sequence. Given the cost function and constraints.
         * If a time limit as been set with setSolverTimeLimit, the method will return after that time with no or the best solution found so far.
         *
         * @param last_open_loop_input an optimal open loop input sequence (i.e. from last iteration) that is used to warm start the solver
         * @param last_open_loop_state an optimal open loop state sequence (i.e. from last iteration) that is used to warm start the solver
         * @param open_loop_input the optimal/suboptimal input sequence found by the solver, depending on the delay compensation the next input is at discrete time step getStepsToCompensateControllerDelay()
         * @param open_loop_state the optimal/suboptimal state sequence found by the solver
         * @return The return state indicates if the solution is optimal, suboptimal or no solution was found
         */
        virtual SOLVER_RETURN
        solve(const Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &last_open_loop_input,
              const Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &last_open_loop_state,
              Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &open_loop_input,
              Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &open_loop_state) const = 0;
    };
};