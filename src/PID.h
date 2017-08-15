#ifndef PID_H
#define PID_H

#include <limits>
#include <functional>
#include <vector>
#include <deque>

using namespace std;

enum twiddle_operation {
    twiddle_ignore,
    twiddle_collect,
    twiddle_decide_if_increase_was_good,
    twiddle_decide_if_decrease_was_good
};

class PID {
public:
    /*
    * Errors
    */
    double p_error{0};
    double i_error{0};
    double d_error{0};

    /**
     * Kp Ki Kd
     */
    double K[3] = {0, 0, 0};
    double dp[3] = {0.1, 0.00001, 0.1};

    double twiddle_cte = 0;
    bool twiddle_active = false;
    deque<twiddle_operation> twiddle_operations;
    unsigned int twiddle_index = 0;
    double best_error = numeric_limits<double>::max();

    PID(double Kp, double Ki, double Kd, bool twiddle_active);

    virtual ~PID();

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    double Kp();

    double Ki();

    double Kd();

};

#endif /* PID_H */
