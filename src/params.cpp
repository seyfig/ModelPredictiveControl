#include "params.h"

params::params(){
	N = 10;
	dt = 0.1;
	ref_v = 55.0;
	ref_cte = 0.0;
	ref_epsi = 0.0;
	c_cte = 2.0;
	c_cte2 = 200.0;
	c_cte3 = 2000.0;
	c_epsi = 800.0;
	c_v = 0.1;
	c_str = 5.0;
	c_a = 0.0;
	c_str_d = 4000.0;
	c_a_d = 0.0;
	cte_limit2 = 1.5;
	cte_limit3 = 3.0;
	c_cte_m1 = 5.0;
	c_cte_mm1 = -4.0;
}
params::~params(){}