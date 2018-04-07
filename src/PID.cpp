#include "PID.h"
#include <math.h>
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    // Init coeff
    K_ = Vector3d(Kp, Ki, Kd); 

    // Init err
    errors_ = Vector3d(0, 0, 0); 

    // init cte
    cte_ = 0.0;

    // set counter
    counter_ = 0;

    // Twiddle
    dp_ = 0.1*K_;
    param_index_ = 2;  // start from 0 after first twiddle loop

    // timestamp per training epoch
    epoch_length_ = 2000;
    training = true;
    best_err_ = 100000000000.0;

}

bool PID::IsEpochEnd() {
    return (counter_%epoch_length_==0)&&training;
}

void PID::UpdateError(double cte) {
    errors_[0] = cte;
    errors_[2] = cte - cte_;
    errors_[1] += cte;
    cte_ = cte; 
    counter_++;

    if (counter_%epoch_length_!=0) {
        sum_err_ += pow(cte,2) ;
    } else {
        
        counter_ = 0;
        if (best_err_ > 10000000) {
            best_err_ = sum_err_;
            std::cout<<K_[0]<<","<<K_[1]<<","<<K_[2]<<";"<<sum_err_/epoch_length_<<std::endl;
        }

        if (!training) { return; }

        if (sum_err_ < best_err_) {
            std::cout<<K_[0]<<","<<K_[1]<<","<<K_[2]<<";"<<sum_err_/epoch_length_<<std::endl;

            best_err_ = sum_err_;
            // the right way, increase step length.
            dp_[param_index_] *= 1.1;
            // next parameter index
            param_index_ = (param_index_ + 1) % 3;
            isDescending = true;
        }else {
            if(isDescending == true) {
                isDescending = false;
            } else {
                // or else slow down the step. update PID.
                K_[param_index_] += dp_[param_index_];
                dp_[param_index_] *= 0.9;
                // next parameter index
                param_index_ = (param_index_ + 1) % 3;
                isDescending = true;
            }
        }
        
        // update PID 
        if(isDescending) {
            K_[param_index_] += dp_[param_index_];
        }else{
            K_[param_index_] -= 2*dp_[param_index_];
        }

        sum_err_ = 0;
        cte_ = 0.0;
        errors_ = Vector3d(0, 0, 0); 
    }

}

double PID::TotalError() {
    return std::min(std::max(-1.0,-K_.dot(errors_)),1.0);
}