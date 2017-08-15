#include "PID.h"

#include <stdexcept>
#include <string>
#include <iostream>

#define TWIDDLE_IGNORE_AT_START 100
#define TWIDDLE_STEPS_TO_COLLECT 300

using namespace std;

PID::~PID() = default;

void PID::UpdateError(double cte) {
    this->cte = cte;
    int_cte += cte;

    double &param = getParamRef(twiddle_index);
    double &error = getErrorRef(twiddle_index);

    if (twiddle_active) {

        if (twiddle_operations.empty()) {
            cout << "------ starting update of parameter " + to_string(twiddle_index) << endl;
            // time to start a new update!
            twiddle_cte = 0;
            param += dp[twiddle_index];
            for (int i = 0; i < TWIDDLE_STEPS_TO_COLLECT; i++) {
                twiddle_operations.push_back(twiddle_collect);
            }
            twiddle_operations.push_back(twiddle_decide_if_increase_was_good);
        }

        twiddle_operation op = twiddle_operations.front();
        twiddle_operations.pop_front();

        double current_error;

        switch (op) {
            case twiddle_ignore:
                cout << "ignore" << endl;
                break;

            case twiddle_collect:
                twiddle_cte += cte * cte;
                break;

            case twiddle_decide_if_increase_was_good:
                current_error = twiddle_cte / TWIDDLE_STEPS_TO_COLLECT;
                if (current_error < error) {
                    error = current_error;
                    dp[twiddle_index] *= 1.1;
                    twiddle_index = (twiddle_index + 1) % 3;
                } else {
                    param -= 2 * dp[twiddle_index];
                    twiddle_cte = 0;
                    for (int i = 0; i < TWIDDLE_STEPS_TO_COLLECT; i++) {
                        twiddle_operations.push_back(twiddle_collect);
                    }
                    twiddle_operations.push_back(twiddle_decide_if_decrease_was_good);
                }
                break;

            case twiddle_decide_if_decrease_was_good:
                current_error = twiddle_cte / TWIDDLE_STEPS_TO_COLLECT;
                if (current_error < error) {
                    error = current_error;
                    dp[twiddle_index] *= 1.1;
                } else {
                    param += dp[twiddle_index];
                    dp[twiddle_index] *= 0.9;
                }
                twiddle_index = (twiddle_index + 1) % 3;
                break;

            default:
                throw runtime_error("unknown operation");
        }
    }
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}

double &PID::getParamRef(unsigned int i) {
    if (i > 2) throw runtime_error("i too large");
    double *paramPointers[] = {&Kp, &Ki, &Kd};
    return *(paramPointers[i]);
}


double &PID::getErrorRef(unsigned int i) {
    if (i > 2) throw runtime_error("i too large");
    double *errorPointers[] = {&p_error, &i_error, &d_error};
    return *(errorPointers[i]);
}

PID::PID(double Kp, double Ki, double Kd, bool twiddle_active)
        : Kp(Kp), Ki(Ki), Kd(Kd), twiddle_active(twiddle_active) {
    for (int i = 0; i < TWIDDLE_IGNORE_AT_START; i++) {
        twiddle_operations.push_back(twiddle_ignore);
    }

}
