#include "PID.h"

#include <stdexcept>
#include <string>
#include <iostream>

#define TWIDDLE_IGNORE 200
#define TWIDDLE_STEPS_TO_COLLECT 500

using namespace std;

PID::~PID() = default;

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    // get currently active twiddle values as refs so we can update them easily :)
    double &current_k = K[twiddle_index];
    double &current_dp = dp[twiddle_index];

    if (twiddle_active) {

        if (twiddle_operations.empty()) {
            cout << "------ starting update of parameter " + to_string(twiddle_index) << endl;
            // time to start a new update!
            twiddle_cte = 0;
            current_k += current_dp;
            for (int i = 0; i < TWIDDLE_IGNORE; i++) {
                twiddle_operations.push_back(twiddle_ignore);
            }
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

            case twiddle_init_best_error:
                cout << "------- best error is initially set to " << to_string(cte) << endl << endl << endl;
                best_error = cte;
                break;

            case twiddle_collect:
                twiddle_cte += cte * cte;
                break;

            case twiddle_decide_if_increase_was_good:
                current_error = twiddle_cte / TWIDDLE_STEPS_TO_COLLECT;
                if (current_error < best_error) {
                    cout << "------ DECIDE: increase of " << to_string(twiddle_index) << " was good" << endl;
                    best_error = current_error;
                    current_dp *= 1.1;
                    nextTwiddle();
                } else {
                    current_k -= 2 * current_dp;
                    twiddle_cte = 0;
                    for (int i = 0; i < TWIDDLE_IGNORE; i++) {
                        twiddle_operations.push_back(twiddle_ignore);
                    }
                    for (int i = 0; i < TWIDDLE_STEPS_TO_COLLECT; i++) {
                        twiddle_operations.push_back(twiddle_collect);
                    }
                    twiddle_operations.push_back(twiddle_decide_if_decrease_was_good);
                }
                break;

            case twiddle_decide_if_decrease_was_good:
                current_error = twiddle_cte / TWIDDLE_STEPS_TO_COLLECT;
                if (current_error < best_error) {
                    cout << "------ DECIDE: decrease of " << to_string(twiddle_index) << " was good" << endl;
                    best_error = current_error;
                    current_dp *= 1.1;
                } else {
                    cout << "------ DECIDE: decrease of " << to_string(twiddle_index) << " was bad" << endl;
                    current_k += current_dp;
                    current_dp *= 0.9;
                }
                nextTwiddle();
                break;

            default:
                throw runtime_error("unknown operation");
        }
    }
}

double PID::TotalError() {
    return best_error;
}

double PID::Kp() {
    return K[0];
}

double PID::Ki() {
    return K[1];
}

double PID::Kd() {
    return K[2];
}

PID::PID(
        double Kp,
        double Ki,
        double Kd,
        bool twiddle_active,
        bool twiddle_p,
        bool twiddle_i,
        bool twiddle_d)
        : twiddle_active(twiddle_active),
          twiddle_p(twiddle_p),
          twiddle_i(twiddle_i),
          twiddle_d(twiddle_d) {
    K[0] = Kp;
    K[1] = Ki;
    K[2] = Kd;

    // let the car drive for a while so we can find how good the provided values actually are
    for (int i = 0; i < 1111; i++) {
        twiddle_operations.push_back(twiddle_ignore);
    }
    twiddle_operations.push_back(twiddle_init_best_error);
}

PID::PID(double Kp, double Ki, double Kd, bool twiddle_active) : PID(Kp, Ki, Kd, twiddle_active, true, true, true) {}

void PID::nextTwiddle() {
    if (!twiddle_active) {
        return;
    }
    if (!twiddle_p && !twiddle_i && !twiddle_d) {
        return;
    }

    bool stay[] = {twiddle_p, twiddle_i, twiddle_d};
    do {
        twiddle_index = (twiddle_index + 1) % 3;
    } while (!stay[twiddle_index]);
}
