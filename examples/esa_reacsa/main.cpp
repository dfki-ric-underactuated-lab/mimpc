#include "sim/VizForces.hpp"
#include "systems/REACSA.hpp"
#include "solvers/SCIPSolver.hpp"
#include "solvers/DrakeSolver.hpp"
#include "solvers/AcadosSolver.hpp"
#include "sim/Simulation.hpp"
#include <math.h>
#include <iostream>
#include "solvers/LimetingSigmaDeltaModulator.hpp"

#include <cnpy.h>

using namespace mimpc;
using namespace mimpc::simulation;
using namespace mimpc::systems;

void do_sim(std::string solver_name, REACSA::StateVec &state_weight, REACSA::StateVec &state_final_weight, REACSA::InputVec &input_weight, std::string &name, double solve_time_limit, REACSA::StateVec init_state = {1.0, -0.5, M_PI, 0.0, 0.1, 0, 0})
{
    std::cout << "Do sim: " << name << "with  " << solver_name << std::endl;
    static constexpr unsigned int N = 20;
    double rw_bound = 150.0 * reacsa_constants::RPM_2_RADPS;
    unsigned int num_break_trusts = 3;

    double controller_dt = 0.01;

    auto reacsa_model = "models/REACSA.urdf";
    Eigen::Vector<double, REACSA::NUM_STATES> target_state = {0, 0, 0, 0, 0, 0, 0};

    constexpr int delay_comp = 0;
    constexpr INTEGRATION_SCHEME integration_scheme = FORWARD_EULER;

    REACSA reacsa;
    std::unique_ptr<Solver<REACSA, N, 1, 2, 3, delay_comp, integration_scheme>> solver;
    LimetingSigmaDeltaModulators<REACSA::NUM_BIN_INPUTS> mod(1.0, 0.1, 0.1, 0.3, 0.2);
    if (solver_name == "acados")
    {
        solver = std::make_unique<AcadosSolver<REACSA, N, 1, 2, 3, delay_comp, integration_scheme>>(
            state_weight,
            state_final_weight,
            input_weight,
            target_state,
            COST_TYPE::L2Quadratic,
            reacsa,
            0.1,
            PARTIAL_CONDENSING_HPIPM,
            N,
            "SPEED",
            1,
            controller_dt,
            mod
        ); // TODO: not all parameters are taken
    }
    else if (solver_name == "scip")
    {
        solver = std::make_unique<SCIPSolver<REACSA, N, 1, 2, 3, delay_comp, integration_scheme>>(state_weight,
                                                                                                  state_final_weight,
                                                                                                  input_weight,
                                                                                                  target_state,
                                                                                                  COST_TYPE::L1Linear,
                                                                                                  reacsa,
                                                                                                  0.1);
        controller_dt = 0.1;
    }
    else if (solver_name == "drake")
    {
        controller_dt = 0.01;
        solver = std::make_unique<DrakeSolver<REACSA, N, 1, 2, 3, delay_comp, integration_scheme>>(
            state_weight,
            state_final_weight,
            input_weight,
            target_state,
            COST_TYPE::L1Linear,
            reacsa,
            0.1,
            controller_dt,
            mod);
    }
    // CONSTRAINTS
    // system on flatfloor const
    // RW Speed Max and min are mixed up! this is because of
    // https://gitlab.esa.int/orl/platforms/reacsa/ReacsaControl/-/issues/93

    using namespace reacsa_constants;

    double v_bound;
    double x_bound;
    reacsa.get_limitcycle_v_bounds(num_break_trusts, v_bound, x_bound);
    double theta_d_bound = reacsa.get_angular_velocity_bound(rw_bound * RPM_2_RADPS);

    double rw_speed_min = -VELOCITY_REACTION_WHEEL_MAX * RPM_2_RADPS;
    double rw_speed_max = -VELOCITY_REACTION_WHEEL_MIN * RPM_2_RADPS;

    double rw_speed_range = (rw_speed_max - rw_speed_min);
    double rw_speed_range_half = rw_speed_range / 2.0;
    double rw_speed_center = rw_speed_min + rw_speed_range_half;

    init_state(6) = rw_speed_center;

    REACSA::StateVec state_const_lb = {-LENGTH_FLATFLOOR_X / 2. + 1.5 * RADIUS_REACSA,
                                       -LENGTH_FLATFLOOR_Y / 2. + 1.5 * RADIUS_REACSA,
                                       -ROTATION_REACSA_MAX * DEG_2_RAD * 3,
                                       -VELOCITY_REACSA_MAX,
                                       -VELOCITY_REACSA_MAX,
                                       -ROTATIONAL_VELOCITY_REACSA_MAX * DEG_2_RAD,
                                       rw_speed_min};
    REACSA::StateVec state_const_ub = {LENGTH_FLATFLOOR_X / 2 - 1.5 * RADIUS_REACSA,
                                       LENGTH_FLATFLOOR_Y / 2. - 1.5 * RADIUS_REACSA,
                                       ROTATION_REACSA_MAX * DEG_2_RAD * 3,
                                       VELOCITY_REACSA_MAX,
                                       VELOCITY_REACSA_MAX,
                                       ROTATIONAL_VELOCITY_REACSA_MAX * DEG_2_RAD,
                                       rw_speed_max};
    REACSA::StateVec state_final_lb = {state_const_lb(0),
                                       state_const_lb(1),
                                       state_const_lb(2),
                                       -v_bound,
                                       -v_bound,
                                       -theta_d_bound,
                                       (rw_speed_center)-rw_bound};
    REACSA::StateVec state_final_ub = {state_const_ub(0),
                                       state_const_ub(1),
                                       state_const_ub(2),
                                       v_bound,
                                       v_bound,
                                       theta_d_bound,
                                       (rw_speed_center) + rw_bound};

    solver->addInputConstraintOnIndex(0, -TORQUE_REACTION_WHEEL_MAX, TORQUE_REACTION_WHEEL_MAX);
    for (unsigned int k = 0; k < REACSA::NUM_STATES; k++)
    {
        solver->addStateConstraintOnIndex(k, state_const_lb(k), state_const_ub(k));
    }
    solver->setSolverTimeLimit(solve_time_limit);
    solver->addStateConstraintOnStep(N, state_final_lb,
                                     state_final_ub);
    // solver.setState(REACSA::StateVec(0.,0.,0.,0.,0.,0.,rw_speed_center));
    // Eigen::Matrix<double,REACSA::NUM_STATES, N+1, Eigen::RowMajor>xout;
    // Eigen::Matrix<double,REACSA::NUM_INPUTS, N, Eigen::RowMajor> uout;
    // solver.solve(uout,xout, uout, xout);

    // exit(0);

    using SolverT = Solver<REACSA, N, 1, 2, 3, delay_comp, integration_scheme>;

    MPC<REACSA, SolverT> mpc(*(solver.get()));
    Simulation<decltype(mpc)> sim(0.0, reacsa_model, mpc, init_state, target_state, REACSA::StateVec::Constant(0.05),
                                  state_const_lb,
                                  state_const_ub, true, systems::reacsa_constants::FORCE_THRUSTER, controller_dt, 1.0);
    sim.simulateToTarget(60.0);

    auto ret = sim.simulate(60.0);
    sim.saveData(name + "_" + solver_name + "_" +std::to_string(solve_time_limit) + ".npz");
    if (ret == -1)
    {
        exit(130);
    }
}

void test_rand_inits(unsigned int num_experiments)
{

    REACSA::StateVec state_weight = {20.0, 20.0, 2., 0.0001, 0.0001, 1.0, 0.0001};
    REACSA::StateVec state_final_weight = {40.0, 40.0, 2.0, 0.001, 0.001, 1.0, 0.001};
    REACSA::InputVec input_weight = {0.1, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

    std::random_device rd; // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());

    // create various initial
    std::uniform_real_distribution<double> x_value(-reacsa_constants::LENGTH_FLATFLOOR_X / 2. + 1.5 * reacsa_constants::RADIUS_REACSA, reacsa_constants::LENGTH_FLATFLOOR_X / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
    std::uniform_real_distribution<double> y_value(-reacsa_constants::LENGTH_FLATFLOOR_Y / 2. + 1.5 * reacsa_constants::RADIUS_REACSA, reacsa_constants::LENGTH_FLATFLOOR_Y / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
    std::uniform_real_distribution<double> theta_value(-M_PI, M_PI);

    for (unsigned int i = 0; i < num_experiments; i++)
    {
        REACSA::StateVec init_state = {x_value(gen), y_value(gen), theta_value(gen), 0.0, 0.0, 0.0, 0.0};
        std::string name = "rand-test-" + std::to_string(i);
        for (auto solver_name : {"drake", "acados","scip"})
        {
            do_sim(solver_name, state_weight, state_final_weight, input_weight, name, 0.1, init_state);
        }
    }
}

void test_pareto()
{

    // Sim test
    REACSA::StateVec state_weight = {1., 1., 0.12, 0.0, 0.0, 0.0, 0.0};
    REACSA::StateVec state_final_weight = state_weight * 10;

    for (double i = 0.15; i <= 0.25; i += 0.003)
    {
        REACSA::InputVec input_weight = {0.0001, i,i,i,i,i,i,i,i};
        std::string name = "test-w-force_" + std::to_string(i);
        do_sim("acados",state_weight, state_final_weight, input_weight, name, 0.1);
        do_sim("scip",state_weight, state_final_weight, input_weight, name, 0.1);
        do_sim("drake",state_weight, state_final_weight, input_weight, name, 0.1);
    }

    for (double i = 0.; i <= 0.5; i += 0.01)
    {
        REACSA::InputVec input_weight = {0.0001, i,i,i,i,i,i,i,i};
        std::string name = "test-w-force_" + std::to_string(i);
        do_sim("scip",state_weight, state_final_weight, input_weight, name, 1.0);
    }



}

int main()
{
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_t thread_handle = pthread_self();
    pthread_setschedparam(thread_handle, SCHED_FIFO, &params);

    test_pareto();
}
