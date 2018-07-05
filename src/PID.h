#include <numeric>
#include <vector>
#include <iostream>

#ifndef PID_H
#define PID_H

class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;
    double cte_prev;

    /*
    * Coefficients
    */ 
    double Kp;
    double Ki;
    double Kd;
    
    // Twiddle parameters //
    bool with_twiddle;
    unsigned int it;
    double cte_best;
    int previous_try;
    int current_p;
    std::vector<double> p;
    std::vector<double> dp;
    
//     std::vector<double> dp;
//     double tolerance = 0.01;

    /*
    * Constructor
    */
    PID(bool with_twiddle);

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);
//     void Init(double Kp, double Ki, double Kd, double e_p, double e_d, double e_i, double cte_prev);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
};

#endif /* PID_H */
