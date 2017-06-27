#ifndef params_H
#define params_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class params {
 public:
  int N;
  double dt;
  double ref_v;
  double ref_cte;
  double ref_epsi;
  double c_cte;
  // Cost multiplier for cte_limit2
  double c_cte2;
  // Cost multiplier for cte_limit3
  double c_cte3;
  double c_epsi;
  double c_v;
  double c_str;
  double c_a;
  double c_str_d;
  double c_a_d;
  // Add cost for cte > cte_limit2
  double cte_limit2;
  // Add cost for cte > cte_limit3
  double cte_limit3;
  // Add cost for cte - (slope * c_cte_mm1)
  double c_cte_m1;
  double c_cte_mm1;

  params();

  virtual ~params();

};

#endif /* params_H */
