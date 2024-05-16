/**
 * @file System.hpp
 * @brief Abstract class to represent system dynamics
 *
 * @author Franek Stark
 * Contact: franek.stark@dfki.de
 */
#pragma once

#include <eigen3/Eigen/Dense>

namespace mimpc {

/**
 * In this library the system Dynamics are represented as \f$ \mathbf{\dot{x}} = \mathbf{A} \mathbf{x} + \mathbf{B} \mathbf{u} \f$, where
 * - system dynamics \f$\mathbf{A}\f$
 * - input dynamics \f$\mathbf{B}\f$
 * - system state is \f$\mathbf{x} \in \mathbb{R}^{\text{num_states}}\f$
 * - system input consists of a continuous and binary part \f$\mathbf{u} = \left[\mathbf{u}_\text{cont}^T,\mathbf{u}^T_\text{bin}\right]^T\f$, \f$\mathbf{u}_\text{cont} \in \mathbb{R}^{\text{num_cont_inputs}}\f$ and \f$\mathbf{u}_\text{cont} \in \left\{0,1\right\}^{\text{num_bin_inputs}}\f$
 *
 * This class represents the system dynamics by offering functions that return the matrices \f$A,B\f$ in order to allow a state dependend linearization.
 * When the MPC needs to know the system state, it will call the functions getA() or getB() to retrieve the full state matrix based on the current state.
 * In order to avoid computational overhead when the systems state has changed the functions updateA() or updateB() are used in order to retrieve only the matrix values that change due to linearization.
 *
 * To define the dynamics of a system, this abstract class has to be by a class which returns the linearized system dynamics as specified. The (SCIPSolver) or (MPC) can than be instantiated with the system dynamics as a template argument.
 *
 * @tparam num_states Number of system states
 * @tparam num_cont_inputs Number of continuous inputs
 * @tparam num_bin_inputs Number of binary inputs
 */
    template<int num_states, int num_cont_inputs, int num_bin_inputs>
    class System {
    public:
        static constexpr auto NUM_STATES = num_states;
        static constexpr auto NUM_CONT_INPUTS = num_cont_inputs;
        static constexpr auto NUM_BIN_INPUTS = num_bin_inputs;
        static constexpr auto NUM_INPUTS = num_bin_inputs + num_cont_inputs;

        using AMatrix = typename Eigen::Matrix<double, num_states, num_states>;
        using BMatrix = typename Eigen::Matrix<double, num_states, num_cont_inputs + num_bin_inputs>;
        using StateVec = typename Eigen::Vector<double, num_states>;
        using InputVec = typename Eigen::Vector<double, num_cont_inputs + num_bin_inputs>;

        /**
         * Linearizes and returns the system dynamics as matrix A
         * @param state state to linearize the system around
         * @return system matrix A
         */
        virtual AMatrix getA(const StateVec &state) const = 0;

        /**
         * Linearizes and returns the system input dynamics as a matrix B
         * @param state state to linearize the system around
         * @return input matrix B
         */
        virtual BMatrix getB(const StateVec &state) const = 0;

        /**
         * Updates the states of the linearized system matrix A based on a new state
         * @param state state to linearize the system around
         * @param changeAval callback function that will called for each matrix element which changes due to linearization around the new state.
         * The callback function will be called with parameters (row index, column index, value). The values that are not updated are assumed to stay as returned by getA()
         */
        virtual void
        updateA(const StateVec &state, std::function<void(unsigned int, unsigned int, double)> &changeAval) const = 0;

        /**
           * Updates the states of the linearized system matrix B based on a new state
           * @param state state to linearize the system around
           * @param changeAval callback function that will called for each matrix element which changes due to linearization around the new state.
           * The callback function will be called with parameter (row index, column index, value). The values that are not updated are assumed to stay as returned by getB()
           */
        virtual void
        updateB(const StateVec &state, std::function<void(unsigned int, unsigned int, double)> &changeBval) const = 0;
    };
}