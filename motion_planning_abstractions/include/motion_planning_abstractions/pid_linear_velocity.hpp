#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

// PID class for getting velocity from position
class PIDLinearVelocity{
    public:
        template <class T>
        PIDLinearVelocity(T p_gain, T i_gain, T d_gain, T k_gain, T error_velocity_iir_alpha, T frequency, T error_velocity_window_size){
            this->P_GAIN_ = double(p_gain);
            this->I_GAIN_ = double(i_gain);
            this->D_GAIN_ = double(d_gain);
            this->K_GAIN_ = double(k_gain);
            this->error_velocity_iir_alpha_ = double(error_velocity_iir_alpha);
            this->frequency_ = double(frequency);
            this->error_velocity_window_size_ = error_velocity_window_size;

            this->error_integral_ = {0.0,0.0,0.0};
            this->error_velocity_ = {0.0,0.0,0.0};
            this->count_ = 0;
        }

        Eigen::Vector3d get_velocity(Eigen::Vector3d current, Eigen::Vector3d target){ // call this function in a loop
            Eigen::Vector3d error = target - current;
            error_array_.push_back(error);
            error_integral_ += error/frequency_;
            error_velocity_ = get_error_velocity();
            return K_GAIN_*((P_GAIN_*error) + (I_GAIN_*error_integral_) + (D_GAIN_*error_velocity_));
        }

        Eigen::Vector3d get_error_velocity(){
            if(error_array_.size() < this->error_velocity_window_size_){
                return Eigen::Vector3d({0.0, 0.0, 0.0});
            }
            else if(error_array_.size() == this->error_velocity_window_size_){
                error_array_.erase(error_array_.begin());
                return (error_velocity_*(1-error_velocity_iir_alpha_)) + 
                ((error_velocity_iir_alpha_)*(error_array_.back() - error_array_.front())*(this->frequency_/this->error_velocity_window_size_));
            }
            else{
                error_array_.erase(error_array_.begin());
                return Eigen::Vector3d({0.0,0.0,0.0});
            }
        }

        void reset_count(){
            this->count_ = 0;
        }

        void reset_error_integral(){
            this->error_integral_ = {0.0,0.0,0.0};
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
        Eigen::Vector3d error_integral_;
        Eigen::Vector3d error_velocity_;

        std::vector<Eigen::Vector3d> error_array_;
};