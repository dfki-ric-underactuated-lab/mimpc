#include "systems/REACSA.hpp"

namespace mimpc::systems {

    REACSA::REACSA() : REACSA(reacsa_constants::MASS_REACSA, reacsa_constants::INERTIA_REACSA,
                              reacsa_constants::INERTIA_REACTION_WHEEL, reacsa_constants::RADIUS_REACSA,
                              reacsa_constants::FORCE_THRUSTER) {

    }

    double REACSA::get_angular_velocity_bound(double rw_speed_final_bound) {
        return get_angular_velocity_bound(reacsa_constants::INERTIA_REACSA, reacsa_constants::INERTIA_REACTION_WHEEL,
                                          ((reacsa_constants::VELOCITY_REACTION_WHEEL_MAX -
                                            reacsa_constants::VELOCITY_REACTION_WHEEL_MIN) *
                                           reacsa_constants::RPM_2_RADPS) / 2.0, rw_speed_final_bound);
    }

    void REACSA::get_limitcycle_v_bounds(unsigned int num_break_thrusts, double &v_bound, double &x_bound) {
        get_limitcycle_v_bounds(num_break_thrusts, reacsa_constants::MASS_REACSA, reacsa_constants::FORCE_THRUSTER,
                                reacsa_constants::TIME_THRUSTER_MIN_ON, reacsa_constants::TIME_THRUSTER_MIN_OFF,
                                reacsa_constants::TIME_THRUSTER_MAX_ON, v_bound, x_bound);
    }

    REACSA::REACSA(double systemM, double bodyIzz, double rwIzz, double r, double fNom)
            : system_m_(systemM), body_Izz_(bodyIzz), rw_Izz_(rwIzz), r_(r), f_nom(fNom) {
        // A matrix is linear and independend from state, hence can be filled out completly
        A_.fill(0);
        A_(Eigen::seq(0, 2), Eigen::seq(3, 5)) = Eigen::Matrix<double, 3, 3>::Identity();
        // B matrice has to be determined by state and hence can only be pre-filled
        B_.fill(0);
        B_(5, 0) = -1.0 / body_Izz_;
        B_(6, 0) = 1.0 / rw_Izz_;
        B_(5, 1) = (f_nom * r_) / body_Izz_;
        B_(5, 2) = -(f_nom * r_) / body_Izz_;
        B_(5, 3) = (f_nom * r_) / body_Izz_;
        B_(5, 4) = -(f_nom * r_) / body_Izz_;
        B_(5, 5) = (f_nom * r_) / body_Izz_;
        B_(5, 6) = -(f_nom * r_) / body_Izz_;
        B_(5, 7) = (f_nom * r_) / body_Izz_;
        B_(5, 8) = -(f_nom * r_) / body_Izz_;
    }

    SystemSized::AMatrix REACSA::getA(const System::StateVec &state) const {
        (void) state;
        return A_;
    }

    SystemSized::BMatrix REACSA::getB(const System::StateVec &state) const {
        auto B = B_;

        std::function<void(unsigned int, unsigned int, double)> chgB = [&B](unsigned int row, unsigned int col,
                                                                            double val) {
            B(row, col) = val;
        };

        updateB(state, chgB);

        return B;
    }

    double
    REACSA::get_angular_velocity_bound(double body_Izz, double rw_Izz, double rw_speed_max,
                                       double rw_speed_final_bound) {
        auto delta_rw_speed = rw_speed_max - rw_speed_final_bound;
        auto delta_S_speed = (rw_Izz * delta_rw_speed) / body_Izz;
        return delta_S_speed;
    }

    void
    REACSA::get_limitcycle_v_bounds(unsigned int num_break_thrusts, double system_m, double f_nom, double t_min_on,
                                    double t_min_off,
                                    double t_max_on, double &v_bound, double &x_bound) {
        v_bound = (f_nom * t_min_on) / system_m;
        x_bound = (f_nom * t_min_on * t_min_off) / system_m + 0.625 * (f_nom * t_min_on * t_min_on) / system_m;

        for (unsigned int i = 0; i < num_break_thrusts; i++) {
            double v = v_bound;
            double x = x_bound;
            v_bound = v + (f_nom * t_max_on) / system_m;
            x_bound = x + -(f_nom * t_max_on * t_max_on) / (2 * system_m) + v * (t_max_on * t_min_off);
        }
    }

    void REACSA::updateA(const System::StateVec &state,
                         std::function<void(unsigned int, unsigned int, double)> &changeAval) const {
        (void) state;
        (void) changeAval;
        //A matrix is linear and independend from state
    }

    void REACSA::updateB(const System::StateVec &state,
                         std::function<void(unsigned int, unsigned int, double)> &changeBval) const {
        // change the values that depend on the state

        auto theta = state(2);
        auto st = sin(theta);
        auto ct = cos(theta);
        auto a = f_nom / system_m_;
        auto sta = st * a;
        auto cta = ct * a;

        changeBval(3, 1, -sta);
        changeBval(3, 2, sta);
        changeBval(3, 3, -cta);
        changeBval(3, 4, cta);
        changeBval(3, 5, sta);
        changeBval(3, 6, -sta);
        changeBval(3, 7, cta);
        changeBval(3, 8, -cta);
        changeBval(4, 1, cta);
        changeBval(4, 2, -cta);
        changeBval(4, 3, -sta);
        changeBval(4, 4, sta);
        changeBval(4, 5, -cta);
        changeBval(4, 6, cta);
        changeBval(4, 7, sta);
        changeBval(4, 8, -sta);

    }
}
