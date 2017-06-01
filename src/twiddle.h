#ifndef Twiddle_H
#define Twiddle_H
#include <vector>
#include "PID.h"

class Twiddle {
public:

  double tolerance;
  int max_iterations;
  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize Twiddle.
  */
  
  void Init(double tolerance, int max_iterations);
  /*
  * Update the PID error variables given cross track error.
  */
  std::vector<double> UpdateParams(const double cte, std::vector<double> params, PID pid);

};

#endif /* Twiddle_H */
