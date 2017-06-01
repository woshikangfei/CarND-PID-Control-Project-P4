#include "twiddle.h"
#include "PID.h"
#include <float.h>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double tolerance, int max_iterations) 
{
	this->tolerance = tolerance;
	this->max_iterations = max_iterations;
}

std::vector<double> Twiddle::UpdateParams(const double cte, std::vector<double> params, PID pid)
{
	std::vector<double> p = params;
    std::vector<double> dp = { 1, 1, 1 };

    int iterations = 0;
    int params_size = params.size();

    double error;
    double best_error;
    double dp_sum = DBL_MAX;
    double Kp = p[0];
	double Ki = p[1];
	double Kd = p[2];

	pid.Init(Kp, Ki, Kd);
	pid.UpdateError(cte);
    best_error = pid.TotalError();

    while (dp_sum > this->tolerance && iterations <= this->max_iterations)
    {
        for (int i = 0; i < params_size; i++)
        {
            p[i] += dp[i];
            
            Kp = p[0];
			Ki = p[1];
			Kd = p[2];
            
            pid.Init(Kp, Ki, Kd);
            pid.UpdateError(cte);
            error = pid.TotalError();

            if (error < best_error)
            {
                best_error = error;
                dp[i] *= 1.1;
            }
            else
            { 
                p[i] -= 2 * dp[i];
                
            	Kp = p[0];
				Ki = p[1];
				Kd = p[2];
	
				pid.Init(Kp, Ki, Kd);
				pid.UpdateError(cte);

                error = pid.TotalError();

                if (error < best_error)
                {
                    best_error = error;
                    dp[i] *= 1.1;
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }

            //cout << "iteration #: " << iterations << endl;	
            //cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
            //cout << "error: " << error << endl;
            //cout << "best_error: " << best_error << endl;
        }

        iterations++;
        dp_sum = std::accumulate(dp.begin(), dp.end(), 0.0);
  
    }
    return p;
}