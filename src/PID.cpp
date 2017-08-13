#include "PID.h"

#include <stdexcept>
#include <string>

using namespace std;

PID::PID() = default;

PID::~PID() = default;

void PID::Init(double Kp, double Ki, double Kd) {
    // TODO: use constructor instead
}

void PID::UpdateError(double cte) {
    this->cte = cte;
    int_cte += cte;

    double param = getParamRef(twiddle_index);
    double error = getErrorRef(twiddle_index);

    // twiddle algorithm
    switch (mode) {
        case twiddle_increase:
            // TODO

            break;
        case twiddle_decrease:
            // TODO
            break;
        default:
            throw runtime_error("unknown twiddle mode");
    }
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}

double PID::GetValue(double cte) {
    double diff_cte = cte - this->cte;
    return -(Kp * cte + Ki * int_cte + Kd * diff_cte);
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

void PID::moveToNextTwiddle() {
    mode = twiddle_increase;
    twiddle_index = (twiddle_index + 1) % 3;
}
