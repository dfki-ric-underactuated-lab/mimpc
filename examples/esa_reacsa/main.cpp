#include "sim/VizForces.hpp"
#include "systems/REACSA.hpp"
#include "solvers/SCIPSolver.hpp"
#include "sim/Simulation.hpp"
#include <math.h>
#include <iostream>

#include <cnpy.h>

using namespace mimpc;
using namespace mimpc::simulation;
using namespace mimpc::systems;


void do_sim(REACSA::StateVec & state_weight, REACSA::StateVec & state_final_weight, REACSA::InputVec & input_weight, std::string & name, REACSA::StateVec init_state = {1.0, -0.5, M_PI, 0.0, 0.1, 0, 0}) {
    std::cout << "Do sim: " << name << std::endl;
    static constexpr unsigned int N = 20;
    double rw_bound = 150.0 * reacsa_constants::RPM_2_RADPS;
    unsigned int num_break_trusts = 1;

    auto reacsa_model = "models/REACSA.urdf";
    Eigen::Vector<double, REACSA::NUM_STATES> target_state = {0, 0, 0, 0, 0, 0, 0};

    constexpr int delay_comp = 1;
    constexpr INTEGRATION_SCHEME integration_scheme = BACKWARD_EULER;

    REACSA reacsa;
    SCIPSolver<REACSA, N, 1, 2, 3, delay_comp, integration_scheme> solver(
            state_weight,
            state_final_weight,
            input_weight,
            target_state,
            COST_TYPE::L1Linear,
            reacsa,
            0.1
    );

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
                                       (rw_speed_center) - rw_bound};
    REACSA::StateVec state_final_ub = {state_const_ub(0),
                                       state_const_ub(1),
                                       state_const_ub(2),
                                       v_bound,
                                       v_bound,
                                       theta_d_bound,
                                       (rw_speed_center) + rw_bound};

    solver.addInputConstraintOnIndex(0, -TORQUE_REACTION_WHEEL_MAX, TORQUE_REACTION_WHEEL_MAX);
    for (unsigned int k = 0; k < REACSA::NUM_STATES; k++){
        solver.addStateConstraintOnIndex(k, state_const_lb(k), state_const_ub(k));
    }
    solver.setSolverTimeLimit(0.1);
    solver.addStateConstraintOnStep(N, state_final_lb,
                                    state_final_ub);
    MPC<REACSA, decltype(solver)> mpc(solver);
    Simulation<decltype(mpc)> sim(0.1, reacsa_model, mpc, init_state, target_state, REACSA::StateVec::Constant(0.05),
                                  state_const_lb,
                                  state_const_ub, true, systems::reacsa_constants::FORCE_THRUSTER);
    sim.simulateToTarget(100.0);
    auto ret = sim.simulate(5.0);
    sim.saveData(name);
    if(ret == -1){
      exit(130);
    }

}


void test_rand_inits(unsigned int num_experiments) {

    REACSA::StateVec state_weight = {20.0, 20.0, 2., 0.0, 0.0, 1.0, 0.0};
    REACSA::StateVec state_final_weight = {40.0, 40.0, 2.0, 0.0, 0.0, 1.0, 0.0};
    REACSA::InputVec input_weight = {0.1, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};


    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());

    // create various initial
    std::uniform_real_distribution<double> x_value(-reacsa_constants::LENGTH_FLATFLOOR_X / 2. + 1.5 * reacsa_constants::RADIUS_REACSA , reacsa_constants::LENGTH_FLATFLOOR_X / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
    std::uniform_real_distribution<double> y_value(-reacsa_constants::LENGTH_FLATFLOOR_Y / 2. + 1.5 * reacsa_constants::RADIUS_REACSA , reacsa_constants::LENGTH_FLATFLOOR_Y / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
    std::uniform_real_distribution<double> theta_value(-M_PI, M_PI);

    for(unsigned int i = 0; i < num_experiments; i++){
        REACSA::StateVec init_state = {x_value(gen), y_value(gen), theta_value(gen), 0.0, 0.0, 0.0, 0.0};
        std::string name = "rand-test-" + std::to_string(i);
        do_sim(state_weight, state_final_weight, input_weight, name,init_state);
    }

}

void test_pareto(){

    // Sim test
    REACSA::StateVec state_weight = {20.0, 20.0, 1.5, 0.0, 0.0, 1.0, 0.0};
    REACSA::StateVec state_final_weight = {40.0, 40.0, 2.5, 0.0, 0.0, 1.0, 0.0};
    REACSA::InputVec input_weight = {0.1, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};


    for(double i = 0.1; i < 3.0; i+=0.2){
        state_weight = {20.0, 20.0, 2.0, 0.0, 0.0, 0.0, 0.0};
        state_final_weight = state_weight;
        input_weight = {0.0, i, i, i, i, i, i, i, i};
        std::string name = "test-thrust-" + std::to_string(i);
        do_sim(state_weight, state_final_weight, input_weight, name);
    }

    for(double i = 0.0; i < 6.0; i+=0.2){
        state_weight = {20.0, 20.0, 2.0, i, i, 0.0, 0.0};
        state_final_weight = state_weight;
        input_weight = {0.0, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
        std::string name = "test-velw-" + std::to_string(i);
        do_sim(state_weight, state_final_weight, input_weight, name);
    }

    for(double i = 0.0; i < 40.0; i+=5.0){
        state_weight = {20.0, 20.0, 2.0, 0.0, 0.0, 0.0, 0.0};
        state_final_weight = state_weight;
        state_final_weight(0) += i;
        state_final_weight(1) += i;

        input_weight = {0.0, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
        std::string name = "test-finalp-" + std::to_string(i);
        do_sim(state_weight, state_final_weight, input_weight, name);
    }

    for(double i = 0.0; i < 4.0; i+=0.5){
        state_weight = {20.0, 20.0, 2.0, i, i, 0.0, 0.0};
        state_final_weight = state_weight;

        input_weight = {i, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
        std::string name = "test-torque-" + std::to_string(i);
        do_sim(state_weight, state_final_weight, input_weight, name);
    }

    for(unsigned int i = 0; i < 100; i++){
        std::string name = "test-sim-" + std::to_string(i);
        do_sim(state_weight, state_final_weight, input_weight, name);
    }

}

int main() {
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_t thread_handle = pthread_self();
    pthread_setschedparam(thread_handle, SCHED_FIFO, &params);

    test_rand_inits(200);

}

