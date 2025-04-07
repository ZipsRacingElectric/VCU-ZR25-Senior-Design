/*
 * qp_matrix.h
 *
 *  Created on: Apr 6, 2025
 *      Author: tetra
 */

#ifndef INC_QP_SOLVER_QP_MATRIX_H_
#define INC_QP_SOLVER_QP_MATRIX_H_

typedef struct {
	double Fx_fl_0;
	double Fx_fr_0;
	double Fx_rl_0;
	double Fx_rr_0;
	double Fy_fl_0;
	double Fy_fr_0;
	double Fy_rl_0;
	double Fy_rr_0;
	double Mz_tv;
	double R;
	double alpha_1;
	double alpha_2;
	double alpha_3;
	double ax_ref;
	double dFx_fl_ds;
	double dFx_fr_ds;
	double dFx_rl_ds;
	double dFx_rr_ds;
	double dFy_fl_ds;
	double dFy_fr_ds;
	double dFy_rl_ds;
	double dFy_rr_ds;
	double delta_fl;
	double delta_fr;
	double delta_rr;
	double lf;
	double lr;
	double mass;
	double s_fl_0;
	double s_fr_0;
	double s_rl_0;
	double s_rr_0;
	double tf;
	double tr;
	double w1;
	double w2;
	double w3;
} QPMatrixParams_t;

void computeQP(QPMatrixParams_t params, double* P, double* q);


#endif /* INC_QP_SOLVER_QP_MATRIX_H_ */
