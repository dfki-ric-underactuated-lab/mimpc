#pragma once

#include "../System.hpp"
#include "REACSA_constants.h"
namespace mimpc::systems {
    using SystemSized = System<7, 1, 8>;

    class REACSA : public SystemSized {
    private:
        const double system_m_;
        const double body_Izz_;
        const double rw_Izz_;
        const double r_;
        const double f_nom;

        SystemSized::AMatrix A_;
        SystemSized::BMatrix B_;
    public:

        REACSA(double systemM, double bodyIzz, double rwIzz, double r, double fNom);

        REACSA();

        virtual ~REACSA() = default;

        SystemSized::AMatrix getA(const SystemSized::StateVec &state) const override;

        SystemSized::BMatrix getB(const SystemSized::StateVec &state) const override;

        void updateA(const SystemSized::StateVec &state,
                     std::function<void(unsigned int, unsigned int, double)> &changeAval) const override;

        void updateB(const SystemSized::StateVec &state,
                     std::function<void(unsigned int, unsigned int, double)> &changeBval) const override;

        static double get_angular_velocity_bound(double rw_speed_final_bound);

        static void get_limitcycle_v_bounds(unsigned int num_break_thrusts, double &v_bound, double &x_bound);

        static double
        get_angular_velocity_bound(double body_Izz, double rw_Izz, double rw_speed_max, double rw_speed_final_bound);

        static void
        get_limitcycle_v_bounds(unsigned int num_break_thrusts, double system_m, double f_nom, double t_min_on,
                                double t_min_off, double t_max_on, double &v_bound, double &x_bound);

    };
}

