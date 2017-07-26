// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HIQP_GUROBI_SOLVER_H
#define HIQP_GUROBI_SOLVER_H

#include <gurobi_c++.h>
#include <hiqp/hiqp_solver.h>

namespace hiqp {
/*! \brief An optimization based solver for a set of stages based on Gurobi.
 *  \author Robert Krug, Marcus A Johansson */
class GurobiSolver : public HiQPSolver {
 public:
  GurobiSolver();
  ~GurobiSolver() noexcept {}

  bool solve(std::vector<double>& solution);

 private:
  GurobiSolver(const GurobiSolver& other) = delete;
  GurobiSolver(GurobiSolver&& other) = delete;
  GurobiSolver& operator=(const GurobiSolver& other) = delete;
  GurobiSolver& operator=(GurobiSolver&& other) noexcept = delete;
  
/*! \brief maintains the accumulated constraints for solving the QP in the p-th stage. After appending constraints, the entries of the slack variables w_ correspond to the previously found solutions and zeros for the current (to be solved) stage */
  struct HQPConstraints {
    HQPConstraints() : n_acc_stage_dims_(0) {}

    void reset(unsigned int n_solution_dims);
    void appendConstraints(const HiQPStage& current_stage);

    unsigned int n_acc_stage_dims_;  ///< number of accumulated dimensions of all the previously solved stages
    unsigned int n_stage_dims_;  ///< number of dimensions of the current stage
    Eigen::VectorXd w_;
    Eigen::VectorXd b_;
    Eigen::MatrixXd B_;
    std::vector<char> constraint_signs_;
  };

  struct QPProblem {
    QPProblem(const GRBEnv& env, HQPConstraints& hqp_constraints,
              unsigned int solution_dims);

    ~QPProblem();

    void setup();
    void solve();
    void getSolution(std::vector<double>& solution);

    GRBModel model_;  // Gurobi model (one per each QP problem is used)
    HQPConstraints& hqp_constraints_;
    unsigned int solution_dims_;

    GRBVar* ddq_;     // objective variables for joint accelerations
    double* lb_ddq_;  // lower bounds for ddq
    double* ub_ddq_;  // upper bounds for ddq

    GRBVar* w_;     // slack variables
    double* lb_w_;  // lower bounds for w
    double* ub_w_;  // upper bounds for w

    double* rhsides_;      // Right-hand-side constants
    GRBLinExpr* lhsides_;  // Left-hand-side expressions
    double* coeff_ddq_;     // Coeffs of dq in LHS expression
    double* coeff_w_;      // Coeffs of w in LHS expression

    GRBConstr* constraints_;  //
  };

  GRBEnv env_;
  unsigned int n_solution_dims_;  // number of solution dimensions
  HQPConstraints hqp_constraints_;
};

}  // namespace hiqp

#endif  // include guard
