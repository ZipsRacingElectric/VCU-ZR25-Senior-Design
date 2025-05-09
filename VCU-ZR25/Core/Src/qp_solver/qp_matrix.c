/* Generated C code for computing QP matrices
   Free parameters: Fx_fl_0, Fx_fr_0, Fx_rl_0, Fx_rr_0, Fy_fl_0, Fy_fr_0, Fy_rl_0, Fy_rr_0, Mz_tv, R, alpha_1, alpha_2, alpha_3, ax_ref, dFx_fl_ds, dFx_fr_ds, dFx_rl_ds, dFx_rr_ds, dFy_fl_ds, dFy_fr_ds, dFy_rl_ds, dFy_rr_ds, delta_fl, delta_fr, delta_rr, lf, lr, mass, s_fl_0, s_fr_0, s_rl_0, s_rr_0, tf, tr, w1, w2, w3
   Outputs: P (matrix) and q (vector) stored in arrays
*/
#include <math.h>
#include "qp_solver/qp_matrix.h"

void computeQP(QPMatrixParams_t params, double* P, double* q) {
	double Fx_fl_0 = params.Fx_fl_0;
	double Fx_fr_0 = params.Fx_fr_0;
	double Fx_rl_0 = params.Fx_rl_0;
	double Fx_rr_0 = params.Fx_rr_0;
	double Fy_fl_0 = params.Fy_fl_0;
	double Fy_fr_0 = params.Fy_fr_0;
	double Fy_rl_0 = params.Fy_rl_0;
	double Fy_rr_0 = params.Fy_rr_0;
	double Mz_tv = params.Mz_tv;
	double R = params.R;
	double alpha_1 = params.alpha_1;
	double alpha_2 = params.alpha_2;
	double alpha_3 = params.alpha_3;
	double ax_ref = params.ax_ref;
	double dFx_fl_ds = params.dFx_fl_ds;
	double dFx_fr_ds = params.dFx_fr_ds;
	double dFx_rl_ds = params.dFx_rl_ds;
	double dFx_rr_ds = params.dFx_rr_ds;
	double dFy_fl_ds = params.dFy_fl_ds;
	double dFy_fr_ds = params.dFy_fr_ds;
	double dFy_rl_ds = params.dFy_rl_ds;
	double dFy_rr_ds = params.dFy_rr_ds;
	double delta_fl = params.delta_fl;
	double delta_fr = params.delta_fr;
	double delta_rr = params.delta_rr;
	double lf = params.lf;
	double lr = params.lr;
	double mass = params.mass;
	double s_fl_0 = params.s_fl_0;
	double s_fr_0 = params.s_fr_0;
	double s_rl_0 = params.s_rl_0;
	double s_rr_0 = params.s_rr_0;
	double tf = params.tf;
	double tr = params.tr;
	double w1 = params.w1;
	double w2 = params.w2;
	double w3 = params.w3;

	// Pre-calculate common sub-expressions
    double cse1 = 1.0/(R*R);
    double cse2 = 1.0/(mass*mass)*w2;
    /* Compute P matrix elements 
        [0,  1,  2,  3,
         4,  5,  6,  7, 
    P =  8,  9, 10, 11,
        12, 13, 14, 15]
    */

    P[0] = (cse1*1.0/(dFx_fl_ds*dFx_fl_ds)*w3*2.0)/alpha_3+(cse1*1.0/(dFx_fl_ds*dFx_fl_ds)*w1*pow(dFx_fl_ds*lf*sin(delta_fl)*2.0+dFy_fl_ds*tf*sin(delta_fl)+dFy_fl_ds*lf*cos(delta_fl)*2.0-dFx_fl_ds*tf*cos(delta_fl),2.0))/(alpha_1*2.0)+(cse1*1.0/(dFx_fl_ds*dFx_fl_ds)*cse2*pow(dFx_fl_ds*cos(delta_fl)-dFy_fl_ds*sin(delta_fl),2.0)*2.0)/alpha_2;
    P[1] = (cse1*w1*(dFx_fl_ds*lf*sin(delta_fl)*2.0+dFy_fl_ds*tf*sin(delta_fl)+dFy_fl_ds*lf*cos(delta_fl)*2.0-dFx_fl_ds*tf*cos(delta_fl))*(dFx_fr_ds*lf*sin(delta_fr)*2.0-dFy_fr_ds*tf*sin(delta_fr)+dFy_fr_ds*lf*cos(delta_fr)*2.0+dFx_fr_ds*tf*cos(delta_fr)))/(alpha_1*dFx_fl_ds*dFx_fr_ds*2.0)+(cse1*cse2*(dFx_fl_ds*cos(delta_fl)-dFy_fl_ds*sin(delta_fl))*(dFx_fr_ds*cos(delta_fr)-dFy_fr_ds*sin(delta_fr))*2.0)/(alpha_2*dFx_fl_ds*dFx_fr_ds);
    P[2] = (cse1*w1*(dFx_fl_ds*lf*sin(delta_fl)*2.0+dFy_fl_ds*tf*sin(delta_fl)+dFy_fl_ds*lf*cos(delta_fl)*2.0-dFx_fl_ds*tf*cos(delta_fl))*(dFx_rl_ds*lr*sin(delta_rr)*2.0-dFy_rl_ds*tr*sin(delta_rr)+dFy_rl_ds*lr*cos(delta_rr)*2.0+dFx_rl_ds*tr*cos(delta_rr))*(-1.0/2.0))/(alpha_1*dFx_fl_ds*dFx_rl_ds)+(cse1*cse2*(dFx_fl_ds*cos(delta_fl)-dFy_fl_ds*sin(delta_fl))*(dFx_rl_ds*cos(delta_rr)-dFy_rl_ds*sin(delta_rr))*2.0)/(alpha_2*dFx_fl_ds*dFx_rl_ds);
    P[3] = (cse1*w1*(dFx_fl_ds*lf*sin(delta_fl)*2.0+dFy_fl_ds*tf*sin(delta_fl)+dFy_fl_ds*lf*cos(delta_fl)*2.0-dFx_fl_ds*tf*cos(delta_fl))*(dFx_rr_ds*lr*sin(delta_rr)*2.0+dFy_rr_ds*tr*sin(delta_rr)+dFy_rr_ds*lr*cos(delta_rr)*2.0-dFx_rr_ds*tr*cos(delta_rr))*(-1.0/2.0))/(alpha_1*dFx_fl_ds*dFx_rr_ds)+(cse1*cse2*(dFx_fl_ds*cos(delta_fl)-dFy_fl_ds*sin(delta_fl))*(dFx_rr_ds*cos(delta_rr)-dFy_rr_ds*sin(delta_rr))*2.0)/(alpha_2*dFx_fl_ds*dFx_rr_ds);
    P[5] = (cse1*1.0/(dFx_fr_ds*dFx_fr_ds)*w3*2.0)/alpha_3+(cse1*1.0/(dFx_fr_ds*dFx_fr_ds)*w1*pow(dFx_fr_ds*lf*sin(delta_fr)*2.0-dFy_fr_ds*tf*sin(delta_fr)+dFy_fr_ds*lf*cos(delta_fr)*2.0+dFx_fr_ds*tf*cos(delta_fr),2.0))/(alpha_1*2.0)+(cse1*1.0/(dFx_fr_ds*dFx_fr_ds)*cse2*pow(dFx_fr_ds*cos(delta_fr)-dFy_fr_ds*sin(delta_fr),2.0)*2.0)/alpha_2;
    P[6] = (cse1*w1*(dFx_fr_ds*lf*sin(delta_fr)*2.0-dFy_fr_ds*tf*sin(delta_fr)+dFy_fr_ds*lf*cos(delta_fr)*2.0+dFx_fr_ds*tf*cos(delta_fr))*(dFx_rl_ds*lr*sin(delta_rr)*2.0-dFy_rl_ds*tr*sin(delta_rr)+dFy_rl_ds*lr*cos(delta_rr)*2.0+dFx_rl_ds*tr*cos(delta_rr))*(-1.0/2.0))/(alpha_1*dFx_fr_ds*dFx_rl_ds)+(cse1*cse2*(dFx_fr_ds*cos(delta_fr)-dFy_fr_ds*sin(delta_fr))*(dFx_rl_ds*cos(delta_rr)-dFy_rl_ds*sin(delta_rr))*2.0)/(alpha_2*dFx_fr_ds*dFx_rl_ds);
    P[7] = (cse1*w1*(dFx_fr_ds*lf*sin(delta_fr)*2.0-dFy_fr_ds*tf*sin(delta_fr)+dFy_fr_ds*lf*cos(delta_fr)*2.0+dFx_fr_ds*tf*cos(delta_fr))*(dFx_rr_ds*lr*sin(delta_rr)*2.0+dFy_rr_ds*tr*sin(delta_rr)+dFy_rr_ds*lr*cos(delta_rr)*2.0-dFx_rr_ds*tr*cos(delta_rr))*(-1.0/2.0))/(alpha_1*dFx_fr_ds*dFx_rr_ds)+(cse1*cse2*(dFx_fr_ds*cos(delta_fr)-dFy_fr_ds*sin(delta_fr))*(dFx_rr_ds*cos(delta_rr)-dFy_rr_ds*sin(delta_rr))*2.0)/(alpha_2*dFx_fr_ds*dFx_rr_ds);
    P[10] = (cse1*1.0/(dFx_rl_ds*dFx_rl_ds)*w3*2.0)/alpha_3+(cse1*1.0/(dFx_rl_ds*dFx_rl_ds)*w1*pow(dFx_rl_ds*lr*sin(delta_rr)*2.0-dFy_rl_ds*tr*sin(delta_rr)+dFy_rl_ds*lr*cos(delta_rr)*2.0+dFx_rl_ds*tr*cos(delta_rr),2.0))/(alpha_1*2.0)+(cse1*1.0/(dFx_rl_ds*dFx_rl_ds)*cse2*pow(dFx_rl_ds*cos(delta_rr)-dFy_rl_ds*sin(delta_rr),2.0)*2.0)/alpha_2;
    P[11] = (cse1*w1*(dFx_rl_ds*lr*sin(delta_rr)*2.0-dFy_rl_ds*tr*sin(delta_rr)+dFy_rl_ds*lr*cos(delta_rr)*2.0+dFx_rl_ds*tr*cos(delta_rr))*(dFx_rr_ds*lr*sin(delta_rr)*2.0+dFy_rr_ds*tr*sin(delta_rr)+dFy_rr_ds*lr*cos(delta_rr)*2.0-dFx_rr_ds*tr*cos(delta_rr)))/(alpha_1*dFx_rl_ds*dFx_rr_ds*2.0)+(cse1*cse2*(dFx_rl_ds*cos(delta_rr)-dFy_rl_ds*sin(delta_rr))*(dFx_rr_ds*cos(delta_rr)-dFy_rr_ds*sin(delta_rr))*2.0)/(alpha_2*dFx_rl_ds*dFx_rr_ds);
    P[15] = (cse1*1.0/(dFx_rr_ds*dFx_rr_ds)*w3*2.0)/alpha_3+(cse1*1.0/(dFx_rr_ds*dFx_rr_ds)*w1*pow(dFx_rr_ds*lr*sin(delta_rr)*2.0+dFy_rr_ds*tr*sin(delta_rr)+dFy_rr_ds*lr*cos(delta_rr)*2.0-dFx_rr_ds*tr*cos(delta_rr),2.0))/(alpha_1*2.0)+(cse1*1.0/(dFx_rr_ds*dFx_rr_ds)*cse2*pow(dFx_rr_ds*cos(delta_rr)-dFy_rr_ds*sin(delta_rr),2.0)*2.0)/alpha_2;
    
    // Matrix is symmetrical so we only have to compute half the elements
    P[4] = P[1];
    P[8] = P[2];
    P[9] = P[6];
    P[12] = P[3];
    P[13] = P[7];
    P[14] = P[11];

    /* Compute q vector elements */
    q[0] = (w1*(lf*(sin(delta_fl)/R+(dFy_fl_ds*cos(delta_fl))/(R*dFx_fl_ds))-(tf*(cos(delta_fl)/R-(dFy_fl_ds*sin(delta_fl))/(R*dFx_fl_ds)))/2.0)*(Mz_tv+lf*cos(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+lf*cos(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)-lr*cos(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)-lr*cos(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds)+(tf*sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds))/2.0-(tf*sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds))/2.0+(tr*sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds))/2.0-(tr*sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/2.0)*-2.0)/alpha_1-(w2*(cos(delta_fl)/R-(dFy_fl_ds*sin(delta_fl))/(R*dFx_fl_ds))*(ax_ref-(sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)+sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)+sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/mass)*2.0)/(alpha_2*mass)-(1.0/(dFx_fl_ds*dFx_fl_ds)*w3*(Fx_fl_0-dFx_fl_ds*s_fl_0)*2.0)/(R*alpha_3);
    q[1] = (w1*(lf*(sin(delta_fr)/R+(dFy_fr_ds*cos(delta_fr))/(R*dFx_fr_ds))+(tf*(cos(delta_fr)/R-(dFy_fr_ds*sin(delta_fr))/(R*dFx_fr_ds)))/2.0)*(Mz_tv+lf*cos(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+lf*cos(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)-lr*cos(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)-lr*cos(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds)+(tf*sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds))/2.0-(tf*sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds))/2.0+(tr*sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds))/2.0-(tr*sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/2.0)*-2.0)/alpha_1-(w2*(cos(delta_fr)/R-(dFy_fr_ds*sin(delta_fr))/(R*dFx_fr_ds))*(ax_ref-(sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)+sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)+sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/mass)*2.0)/(alpha_2*mass)-(1.0/(dFx_fr_ds*dFx_fr_ds)*w3*(Fx_fr_0-dFx_fr_ds*s_fr_0)*2.0)/(R*alpha_3);
    q[2] = (w1*(lr*(sin(delta_rr)/R+(dFy_rl_ds*cos(delta_rr))/(R*dFx_rl_ds))+(tr*(cos(delta_rr)/R-(dFy_rl_ds*sin(delta_rr))/(R*dFx_rl_ds)))/2.0)*(Mz_tv+lf*cos(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+lf*cos(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)-lr*cos(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)-lr*cos(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds)+(tf*sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds))/2.0-(tf*sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds))/2.0+(tr*sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds))/2.0-(tr*sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/2.0)*2.0)/alpha_1-(w2*(cos(delta_rr)/R-(dFy_rl_ds*sin(delta_rr))/(R*dFx_rl_ds))*(ax_ref-(sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)+sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)+sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/mass)*2.0)/(alpha_2*mass)-(1.0/(dFx_rl_ds*dFx_rl_ds)*w3*(Fx_rl_0-dFx_rl_ds*s_rl_0)*2.0)/(R*alpha_3);
    q[3] = (w1*(lr*(sin(delta_rr)/R+(dFy_rr_ds*cos(delta_rr))/(R*dFx_rr_ds))-(tr*(cos(delta_rr)/R-(dFy_rr_ds*sin(delta_rr))/(R*dFx_rr_ds)))/2.0)*(Mz_tv+lf*cos(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+lf*cos(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)-lr*cos(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)-lr*cos(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds)+(tf*sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds))/2.0-(tf*sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds))/2.0+(tr*sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds))/2.0-(tr*sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/2.0)*2.0)/alpha_1-(w2*(cos(delta_rr)/R-(dFy_rr_ds*sin(delta_rr))/(R*dFx_rr_ds))*(ax_ref-(sin(delta_fl)*(Fy_fl_0-(Fx_fl_0*dFy_fl_ds)/dFx_fl_ds)+sin(delta_fr)*(Fy_fr_0-(Fx_fr_0*dFy_fr_ds)/dFx_fr_ds)+sin(delta_rr)*(Fy_rl_0-(Fx_rl_0*dFy_rl_ds)/dFx_rl_ds)+sin(delta_rr)*(Fy_rr_0-(Fx_rr_0*dFy_rr_ds)/dFx_rr_ds))/mass)*2.0)/(alpha_2*mass)-(1.0/(dFx_rr_ds*dFx_rr_ds)*w3*(Fx_rr_0-dFx_rr_ds*s_rr_0)*2.0)/(R*alpha_3);
}
