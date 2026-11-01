#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

#define USE_CODE_GEN 1

int main(){
  // Use Acado
  USING_NAMESPACE_ACADO

  // system variables
  DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  DifferentialState     f1_w, f1_x, f1_y, f1_z;
  DifferentialState     r1;
  DifferentialState     s;
  DifferentialState     sv1_accumulator;
  DifferentialState     sv2_accumulator;
  Control               T, w_x, w_y, w_z;
  Control               ds;
  Control               sv1; // horizontal FOV slack variable
  Control               sv2; // vertical FOV slack variable
  DifferentialEquation  f;
  Function              h, hN;

  const double t_start = 0.0;     // Initial time [s]
  const double dt = 0.05;         // Discretization time [s]
  const int N = 10;               // Number of forward recursion intervals
  const double t_end = N * dt;    // Time horizon [s]
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double sv_min = 0.0;    // slack variable for visibility constraint
  const double sv_max = 1000.0; // slack variable for visibility constraint
  const double r1_min = 0.1;
  const double r1_max = 100.0;
  const double epsilon = 1e-2;
  const double nz_min = 0.05;

#ifndef USE_CODE_GEN

#else
  // 仅相机姿态进 cost（用于把世界系速度转到相机系）。
  OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
  // 短时定速目标在世界系下的速度，用来构造 v_rel = v_uav - v_target。
  OnlineData            target_v_x, target_v_y, target_v_z;
  // Runtime closing-speed hard limit and optimized-state rate limit. The LOS
  // scheduled speed is supplied through the LSQ reference and is therefore
  // frozen over each solve horizon.
  OnlineData            max_intercept_speed, max_delta_intercept_speed;
  // 运行期执行器限幅。
  OnlineData            T_min_limit, T_max_limit;
  OnlineData            w_x_limit, w_y_limit, w_z_limit;
  // Current UAV-to-target line in world frame for terminal alignment.
  OnlineData            line_p_x, line_p_y, line_p_z;
  OnlineData            line_n_x, line_n_y, line_n_z;
  // Runtime time scaling for effective horizon shortening.
  OnlineData            time_scale;
  // Normalized image-plane center and half-width/height used by L_vis.
  OnlineData            img_center_u, img_center_v;
  OnlineData            img_radius_u, img_radius_v;
  // Runtime ZYX roll-angle safety limit. The nonlinear constraints use
  // tan(max_roll) so the model does not need atan2 inside the OCP.
  OnlineData            tan_max_roll;
#endif

  // Relative translational velocity between the UAV and a constant-velocity target, expressed in world frame.
  // The original code assumed a stationary landmark. For interception, replace v_B^w by v_B^w - v_T^w.
  IntermediateState v_rel_w_x = v_x - target_v_x;
  IntermediateState v_rel_w_y = v_y - target_v_y;
  IntermediateState v_rel_w_z = v_z - target_v_z;

  // 目标相对 UAV 的速度，在相机坐标系下沿相机 x 轴的分量
  IntermediateState vc_x = v_rel_w_z*((2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)*(2*q_x*q_x + 2*q_y*q_y - 1) + (2*q_w*q_y - 2*q_x*q_z)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) + (2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)*(2*q_w*q_x + 2*q_y*q_z)) - v_rel_w_x*((2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)*(2*q_w*q_y + 2*q_x*q_z) + (2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)*(2*q_w*q_z - 2*q_x*q_y) - (2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1)*(2*q_y*q_y + 2*q_z*q_z - 1)) - v_rel_w_y*((2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)*(2*q_x*q_x + 2*q_z*q_z - 1) + (2*q_w*q_z + 2*q_x*q_y)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) - (2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)*(2*q_w*q_x - 2*q_y*q_z));
  IntermediateState vc_y = v_rel_w_x*((2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)*(2*q_y*q_y + 2*q_z*q_z - 1) + (2*q_w*q_z - 2*q_x*q_y)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) + (2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z)*(2*q_w*q_y + 2*q_x*q_z)) - v_rel_w_y*((2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z)*(2*q_w*q_x - 2*q_y*q_z) + (2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)*(2*q_w*q_z + 2*q_x*q_y) - (2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1)*(2*q_x*q_x + 2*q_z*q_z - 1)) - v_rel_w_z*((2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z)*(2*q_x*q_x + 2*q_y*q_y - 1) + (2*q_w*q_x + 2*q_y*q_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) - (2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)*(2*q_w*q_y - 2*q_x*q_z));
  IntermediateState vc_z = v_rel_w_y*((2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*q_x*q_x + 2*q_z*q_z - 1) + (2*q_w*q_x - 2*q_y*q_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) + (2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*q_w*q_z + 2*q_x*q_y)) - v_rel_w_x*((2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*q_y*q_y + 2*q_z*q_z - 1) + (2*q_w*q_y + 2*q_x*q_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) - (2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*q_w*q_z - 2*q_x*q_y)) - v_rel_w_z*((2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*q_w*q_x + 2*q_y*q_z) + (2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*q_w*q_y - 2*q_x*q_z) - (2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1)*(2*q_x*q_x + 2*q_y*q_y - 1));

  // 相机坐标系中的绕 x 轴角速度分量
  IntermediateState wc_x = w_y*(2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y) - w_z*(2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z) - w_x*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1);
  IntermediateState wc_y = w_z*(2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z) - w_x*(2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y) - w_y*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1);
  IntermediateState wc_z = w_x*(2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z) - w_y*(2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z) - w_z*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1);
 
  // 相对距离沿视线方向的变化速度
  IntermediateState rdot1 = (vc_z*(2*f1_x*f1_x + 2*f1_y*f1_y - 1))/1.0 - (vc_x*(2*f1_w*f1_y + 2*f1_x*f1_z))/1.0 + (vc_y*(2*f1_w*f1_x - 2*f1_y*f1_z))/1.0;
  
  // System Dynamics
  f << dot(p_x) ==  time_scale * v_x;
  f << dot(p_y) ==  time_scale * v_y;
  f << dot(p_z) ==  time_scale * v_z;
  f << dot(q_w) ==  time_scale * 0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  time_scale * 0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  time_scale * 0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  time_scale * 0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  time_scale * (2 * ( q_w * q_y + q_x * q_z ) * T);
  f << dot(v_y) ==  time_scale * (2 * ( q_y * q_z - q_w * q_x ) * T);
  f << dot(v_z) ==  time_scale * (( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T - g_z);
  
  IntermediateState nx;
  IntermediateState ny;
  IntermediateState nz;

  nx = 2*f1_w*f1_y + 2*f1_x*f1_z;
  ny = 2*f1_y*f1_z - 2*f1_w*f1_x;
  nz = 1 - 2*f1_x*f1_x - 2*f1_y*f1_y;

  IntermediateState Omega_x;
  IntermediateState Omega_y;
  IntermediateState Omega_z;
  // Horizontal (spin-free) lift of the bearing motion. The camera angular
  // velocity parallel to n changes only the quaternion's gauge, not R(f1)e3.
  IntermediateState wc_along_n = nx*wc_x + ny*wc_y + nz*wc_z;

  Omega_x = -(ny*vc_z - nz*vc_y)/(r1 + epsilon) - wc_x + nx*wc_along_n;
  Omega_y = -(nz*vc_x - nx*vc_z)/(r1 + epsilon) - wc_y + ny*wc_along_n;
  Omega_z = -(nx*vc_y - ny*vc_x)/(r1 + epsilon) - wc_z + nz*wc_along_n;

  IntermediateState f1_w_dot =
      -0.5 * (Omega_x*f1_x + Omega_y*f1_y + Omega_z*f1_z);
  IntermediateState f1_x_dot =
       0.5 * (Omega_x*f1_w + Omega_y*f1_z - Omega_z*f1_y);
  IntermediateState f1_y_dot =
       0.5 * (-Omega_x*f1_z + Omega_y*f1_w + Omega_z*f1_x);
  IntermediateState f1_z_dot =
       0.5 * (Omega_x*f1_y - Omega_y*f1_x + Omega_z*f1_w);

  f << dot(f1_w) == time_scale * f1_w_dot;
  f << dot(f1_x) == time_scale * f1_x_dot;
  f << dot(f1_y) == time_scale * f1_y_dot;
  f << dot(f1_z) == time_scale * f1_z_dot;

  f << dot(r1) == time_scale * rdot1;
  f << dot(s) == time_scale * ds;
  // ACADO codegen drops controls that do not enter the dynamics. These unused
  // accumulator states keep sv1/sv2 as optimized controls while the FOV
  // constraints below use sv1/sv2 directly as slack variables.
  f << dot(sv1_accumulator) == time_scale * sv1;
  f << dot(sv2_accumulator) == time_scale * sv2;

  // Visual objective in the normalized image plane:
  // e_vis = diag(rho)^-1 * ([nx/nz, ny/nz]^T - pi_c).
  IntermediateState img_u = nx / (nz + epsilon);
  IntermediateState img_v = ny / (nz + epsilon);
  IntermediateState e_vis_u = (img_u - img_center_u) / img_radius_u;
  IntermediateState e_vis_v = (img_v - img_center_v) / img_radius_v;
  IntermediateState terminal_img_err_u = e_vis_u;
  IntermediateState terminal_img_err_v = e_vis_v;

  // ZYX roll satisfies roll = atan2(R32, R33).  Constraining
  // |R32| <= tan(max_roll) * R33 avoids atan2 while keeping the vehicle on
  // the upright Euler branch.  A small fixed positive R33 margin prevents
  // the optimizer from sitting exactly on the +/-90 degree singularity.
  IntermediateState roll_s = 2.0 * (q_y*q_z + q_w*q_x);  // R32
  IntermediateState roll_c = 1.0 - 2.0*q_x*q_x - 2.0*q_y*q_y;  // R33

  // Spherical bearing angular motion: e_jit = n_t x n_dot_t.
  IntermediateState nx_dot =
      2.0 * (f1_w_dot * f1_y + f1_w * f1_y_dot +
             f1_x_dot * f1_z + f1_x * f1_z_dot);
  IntermediateState ny_dot =
      2.0 * (f1_y_dot * f1_z + f1_y * f1_z_dot -
             f1_w_dot * f1_x - f1_w * f1_x_dot);
  IntermediateState nz_dot =
      -4.0 * f1_x * f1_x_dot - 4.0 * f1_y * f1_y_dot;
  IntermediateState e_jit_x = ny * nz_dot - nz * ny_dot;
  IntermediateState e_jit_y = nz * nx_dot - nx * nz_dot;
  IntermediateState e_jit_z = nx * ny_dot - ny * nx_dot;

  // Interception velocity objective in world frame. The optimized speed state s
  // tracks a runtime LSQ reference computed from the current measured LOS. It
  // is intentionally not recomputed from predicted LOS states, preventing the
  // optimizer from lowering its own speed target by creating image error.
  IntermediateState vel_err_x = v_rel_w_x - s * line_n_x;
  IntermediateState vel_err_y = v_rel_w_y - s * line_n_y;
  IntermediateState vel_err_z = v_rel_w_z - s * line_n_z;
  IntermediateState thrust_err = T - g_z;

  IntermediateState terminal_line_dx = p_x - line_p_x;
  IntermediateState terminal_line_dy = p_y - line_p_y;
  IntermediateState terminal_line_dz = p_z - line_p_z;
  IntermediateState terminal_line_cx =
      terminal_line_dy * line_n_z - terminal_line_dz * line_n_y;
  IntermediateState terminal_line_cy =
      terminal_line_dz * line_n_x - terminal_line_dx * line_n_z;
  IntermediateState terminal_line_cz =
      terminal_line_dx * line_n_y - terminal_line_dy * line_n_x;
  IntermediateState terminal_line_dist =
      sqrt(terminal_line_cx * terminal_line_cx +
           terminal_line_cy * terminal_line_cy +
           terminal_line_cz * terminal_line_cz + epsilon * epsilon);

  h << vel_err_x
    << vel_err_y
    << vel_err_z
    << e_vis_u
    << e_vis_v
    << e_jit_x
    << e_jit_y
    << e_jit_z
    << s
    << thrust_err << w_x << w_y << w_z
    << ds
    << sv1 << sv2;

  // Terminal LSQ term: finish the horizon on the current UAV-target line with
  // the target centered in the normalized image plane.
  hN << terminal_line_dist
     << terminal_img_err_u
     << terminal_img_err_v;

  // Running cost weight matrix
  // h = [vel_err_x, vel_err_y, vel_err_z, e_vis_u, e_vis_v,
  //      e_jit_x, e_jit_y, e_jit_z, speed, T-g, wx, wy, wz, ds,
  //      slack_u, slack_v]
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 10;    // relative velocity error along world line x
  Q(1,1) = 10;    // relative velocity error along world line y
  Q(2,2) = 10;    // relative velocity error along world line z
  Q(3,3) = 20;    // normalized horizontal image-plane residual
  Q(4,4) = 20;    // normalized vertical image-plane residual
  Q(5,5) = 5;     // bearing angular motion x penalty
  Q(6,6) = 5;     // bearing angular motion y penalty
  Q(7,7) = 5;     // bearing angular motion z penalty
  Q(8,8) = 50;    // optimized closing speed tracks the frozen runtime reference
  Q(9,9) = 1;     // thrust deviation from hover regularization
  Q(10,10) = 1;   // body-rate x regularization
  Q(11,11) = 1;   // body-rate y regularization
  Q(12,12) = 1;   // body-rate z regularization
  Q(13,13) = 1;   // closing speed rate regularization
  Q(14,14) = 500; // horizontal FOV slack penalty
  Q(15,15) = 500; // vertical FOV slack penalty

  // End cost weight matrix
  // hN = [terminal_line_dist, terminal_img_err_u, terminal_img_err_v]
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = 50;
  QN(1,1) = 20;
  QN(2,2) = 20;

  // Runtime code replaces the speed entry with the current LOS-scheduled
  // reference; the remaining entries stay at zero.
  DVector r(h.getDim());
  r.setZero();

  DVector rN(hN.getDim());
  rN.setZero();

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );

#ifndef USE_CODE_GEN
  // For analysis, set references.
  ocp.minimizeLSQ( Q, h, r );
  ocp.minimizeLSQEndTerm( QN, hN, rN );
#else
  // For code generation, references are set during run time.
  BMatrix Q_sparse(h.getDim(), h.getDim());
  Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim());
  QN_sparse.setIdentity();
  ocp.minimizeLSQ( Q_sparse, h);
  ocp.minimizeLSQEndTerm( QN_sparse, hN );
#endif

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(T_min_limit - T <= 0.0);
  ocp.subjectTo(T - T_max_limit <= 0.0);
  ocp.subjectTo(-w_x - w_x_limit <= 0.0);
  ocp.subjectTo(w_x - w_x_limit <= 0.0);
  ocp.subjectTo(-w_y - w_y_limit <= 0.0);
  ocp.subjectTo(w_y - w_y_limit <= 0.0);
  ocp.subjectTo(-w_z - w_z_limit <= 0.0);
  ocp.subjectTo(w_z - w_z_limit <= 0.0);
  ocp.subjectTo(-ds - max_delta_intercept_speed <= 0.0);
  ocp.subjectTo(ds - max_delta_intercept_speed <= 0.0);
  ocp.subjectTo(sv_min <= sv1 <= sv_max);
  ocp.subjectTo(sv_min <= sv2 <= sv_max);

  ocp.subjectTo(r1_min <= r1);
  ocp.subjectTo(-s <= 0.0);
  ocp.subjectTo(s - max_intercept_speed <= 0.0);
  ocp.subjectTo(nz_min - nz <= 0.0);

  ocp.subjectTo(img_u - sv1 - img_center_u - img_radius_u <= 0.0);
  ocp.subjectTo(img_center_u - img_radius_u - img_u - sv1 <= 0.0);
  ocp.subjectTo(img_v - sv2 - img_center_v - img_radius_v <= 0.0);
  ocp.subjectTo(img_center_v - img_radius_v - img_v - sv2 <= 0.0);

  constexpr double kMinRollDenominator = 1e-3;
  ocp.subjectTo(roll_s - tan_max_roll * roll_c <= 0.0);
  ocp.subjectTo(-roll_s - tan_max_roll * roll_c <= 0.0);
  ocp.subjectTo(kMinRollDenominator - roll_c <= 0.0);
  // ACADO path constraints cover shooting nodes 0..N-1. Constrain xN
  // explicitly so the optimizer cannot postpone a roll violation to the
  // terminal state.
  ocp.subjectTo(AT_END, roll_s - tan_max_roll * roll_c <= 0.0);
  ocp.subjectTo(AT_END, -roll_s - tan_max_roll * roll_c <= 0.0);
  ocp.subjectTo(AT_END, kMinRollDenominator - roll_c <= 0.0);

  ocp.setNOD(26);

#ifndef USE_CODE_GEN
  // Set initial state
  ocp.subjectTo( AT_START, p_x ==  0.0 );
  ocp.subjectTo( AT_START, p_y ==  0.0 );
  ocp.subjectTo( AT_START, p_z ==  1.0 );
  ocp.subjectTo( AT_START, q_w ==  1.0 );
  ocp.subjectTo( AT_START, q_x ==  0.0 );
  ocp.subjectTo( AT_START, q_y ==  0.0 );
  ocp.subjectTo( AT_START, q_z ==  0.0 );
  ocp.subjectTo( AT_START, v_x ==  0.0 );
  ocp.subjectTo( AT_START, v_y ==  0.0 );
  ocp.subjectTo( AT_START, v_z ==  0.0 );

  // Define an algorithm to solve it.
  OptimizationAlgorithm algorithm(ocp);
  algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
  algorithm.set( KKT_TOLERANCE, 1e-3 );
  algorithm.solve();
#else
  // For code generation, we can set some properties.
  // The main reason for a setting is given as comment.
  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
  mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
  mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
  mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
  mpc.set(NUM_INTEGRATOR_STEPS,   N);
  mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
  mpc.set(HOTSTART_QP,            YES);
  mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
  mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

  // Do not generate tests, makes or matlab-related interfaces.
  mpc.set( GENERATE_TEST_FILE,          NO);
  mpc.set( GENERATE_MAKE_FILE,          NO);
  mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
  mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

  // Finally, export everything.
  if(mpc.exportCode("so3_quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );
  mpc.printDimensionsQP( );
#endif

  return EXIT_SUCCESS;

}
