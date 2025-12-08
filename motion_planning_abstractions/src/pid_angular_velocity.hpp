#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class PIDAngularVelocity{
    public:
        PIDAngularVelocity(){}

        template <class T>
        PIDAngularVelocity(T p_gain, T i_gain, T d_gain, T k_gain, T error_velocity_iir_alpha, T frequency, T error_velocity_window_size){
            this->P_GAIN_ = double(p_gain);
            this->I_GAIN_ = double(i_gain);
            this->D_GAIN_ = double(d_gain);
            this->K_GAIN_ = double(k_gain);
            this->error_velocity_iir_alpha_ = double(error_velocity_iir_alpha);
            this->frequency_ = double(frequency);
            this->error_velocity_window_size_ = error_velocity_window_size;

            this->error_integral_ = {0.0,0.0,0.0,1.0};
            this->error_velocity_ = {0.0,0.0,0.0,1.0};
            this->count_ = 0;
        }

        Eigen::Vector3d get_velocity(Eigen::Quaterniond current, Eigen::Quaterniond target){ // call this function in a loop            
            Eigen::Quaterniond error = target*current.inverse();
            Eigen::AngleAxisd error_angle_axis(error);
            auto angular_velocity = (K_GAIN_*P_GAIN_*error_angle_axis.angle())*error_angle_axis.axis();
            return angular_velocity;
        }

        void reset_count(){
            this->count_ = 0;
        }

        void clear_error_array(){
            this->error_array_.clear();
        }

    private:
        double P_GAIN_;
        double I_GAIN_;
        double D_GAIN_;
        double K_GAIN_;
        double error_velocity_iir_alpha_; // 0 to 1, 0 means trust the unfiltered data more.
        double frequency_;

        u_int64_t count_;
        u_int64_t error_velocity_window_size_;
        Eigen::Quaterniond error_integral_;
        Eigen::Quaterniond error_velocity_;

        std::vector<Eigen::Quaterniond> error_array_;
};

