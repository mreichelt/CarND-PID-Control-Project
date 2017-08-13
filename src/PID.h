#ifndef PID_H
#define PID_H

enum twiddle_mode {
    twiddle_increase,
    twiddle_decrease
};

class PID {
public:
    /*
    * Errors
    */
    double p_error{1};
    double i_error{1};
    double d_error{1};

    /*
    * Coefficients
    */
    double Kp{0};
    double Ki{0};
    double Kd{0};

    double cte{0};
    double int_cte{0};

    unsigned int twiddle_index = 0;
    twiddle_mode mode = twiddle_increase;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    double GetValue(double cte);

    double& getParamRef(unsigned int index);

    double& getErrorRef(unsigned int index);

    void moveToNextTwiddle();
};

#endif /* PID_H */
