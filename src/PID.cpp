#include "PID.h"

using namespace std;


// Constructor //
PID::PID(bool with_twiddle) 
{
    this->with_twiddle = with_twiddle;
}


// Destructor //
PID::~PID() 
{}

void PID::Init(double Kp, double Ki, double Kd)
// void PID::Init(double Kp, double Ki, double Kd, double e_p, double e_d, double e_i, double cte_prev) 
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    this->p_error = 0.0;
    this->d_error = 0.0;
    this->i_error = 0.0;
    this->cte_prev = 0.0;
//     this->p_error = e_p;
//     this->d_error = e_d;
//     this->i_error = e_i;
//     this->cte_prev = cte_prev;
    
    // Twiddle parameters //
    if (this->with_twiddle)
    {
	this->it = 0;
	this->previous_try = 0;
	this->current_p = -1;
	this->p = {0.0, 0.0, 0.0};
	this->dp = {1.0, 1.0, 1.0};
    }
}


void PID::UpdateError(double cte) 
{

    if (this->with_twiddle)
    {
	
	// For the first iteration, initialise the cte_best parameter //
	if (this->it == 0)
	{
	    this->cte_best = cte;
	    this->current_p = (this->current_p + 1) % 3;
	    this->previous_try = 1;
	    this->p[this->current_p] += this->dp[this->current_p];
	}
	// Check which parameter optimisation direction was chosen the previous run, if any //
	else
	{
	    // The last iteration increased the parameter //
	    if (this->previous_try == 1)
	    {
		// If increasing the parameter improved the result //
		if (cte < this->cte_best)
		{
		    this->cte_best = cte;
		    this->dp[this->current_p] *= 1.1;
		    
		    this->current_p = (this->current_p + 1) % 3;
		    this->previous_try = 1;
		    this->p[this->current_p] += this->dp[this->current_p];
		}
		// If increasing the parameter deteriorated the result //
		else
		{
		    this->p[this->current_p] -= 2 * this->dp[this->current_p];
		    this->previous_try = -1;
		}	
	    }
	    else 
	    {   
		// The last iteration decreased the parameter //
		if (previous_try == -1)
		{
		    // If decreasing the parameter improved the result //
		    if (cte < this->cte_best)
		    {
			this->cte_best = cte;
			this->dp[this->current_p] *= 1.1;
			
			this->current_p = (this->current_p + 1) % 3;
			this->previous_try = 1;
			this->p[this->current_p] += this->dp[this->current_p];
		    }
		    // If decreasing the parameter deteriorated the result //
		    else
		    {
			// Revert the parameter back to the original value //
			this->p[this->current_p] += this->dp[this->current_p];
			this->dp[this->current_p] *= 0.9;
			
			this->current_p = (this->current_p + 1) % 3;
			this->previous_try = 1;
			this->p[this->current_p] += this->dp[this->current_p];
		    }
		}
	    }
	}
	
	this->Kp = this->p[0];
	this->Ki = this->p[1];
	this->Kd = this->p[2];
	std::cout << "Kp: " << this->Kp << ", Ki: " << this->Ki << ", Kd: " << this->Kd  << "\n";
	
	this->it++;
	
    } // End of using twiddle //
    

    // Proportional error //
    this->p_error = cte;

    // Differential error //
    if (this->cte_prev == 0.0) // (First pass through)
	{this->d_error = 0.0;}
    else
	{this->d_error = cte - this->cte_prev;}
    this->cte_prev = cte;
    
    // Integral error //
    this->d_error += cte;
    
    
    
}


double PID::TotalError() {
    return 0.0;
}


// void PID::Twiddle(double tolerance)
// {
//     // Initialise the twiddle parameters //
//     std::vector<double> p = {this->Kp, this->Ki, this->Kd};
//     std::vector<double> dp = {0.1, 0.1, 0.1};
//     std::vector<double> error_best = {this->p_error, this->d_error, this->i_error};
// //     int it = 0;
//     while ( tolerance < (p[0]+p[1]+p[2]) )
//     {
// // 	it++;
// 	for (int i=0; i<p.length(); i++)
// 	{
// 	    p[i] += dp[i];
// 	    PID param_tester;
// 	    param_tester.Init(p[0], p[1], p[2], this->p_error, this->d_error, this->i_error, this->cte_prev);
// 	    if 
// 	}
//     }
//     
// }