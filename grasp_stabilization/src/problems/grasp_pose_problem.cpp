#include <ll4ma_opt_utils

// input: object pose w.r.t. world, contact points, surface normal at contact points, 
// o_g= o_R_w*w_g
// min_{o_R_w,f_s,f_i} || f_s || +   || m*o_g+\sum f_i|| + || r_c x m*o_g +\sum_i r_i x f_i||
// s.t.  f_i,f_s \in FC
// o_R_w

