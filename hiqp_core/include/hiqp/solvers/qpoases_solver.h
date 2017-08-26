// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2017 Chittaranjan Srinivas Swaminathan
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


#pragma once
#include <hiqp/hiqp_solver.h>
#include <qpOASES.hpp>
#include <memory>

using qpOASES::real_t;

namespace hiqp {

class qpOASESSolver : public HiQPSolver {
 public:
  qpOASESSolver();
  ~qpOASESSolver() noexcept {}

  bool solve(std::vector<double>& solution);

 private:
  qpOASESSolver(const qpOASESSolver& other) = delete;
  qpOASESSolver(qpOASESSolver&& other) = delete;
  qpOASESSolver& operator=(const qpOASESSolver& other) = delete;
  qpOASESSolver& operator=(qpOASESSolver&& other) noexcept = delete;

  struct HiQPConstraints {
    HiQPConstraints() : n_acc_stage_dims_(0) {}

    void reset(unsigned int n_solution_dims);
    void appendConstraints(const HiQPStage& current_stage);

    unsigned int n_acc_stage_dims_;  // number of accumulated dimensions of all
                                     // the previously solved stages
    unsigned int n_stage_dims_;  // number of dimensions of the current stage
    Eigen::VectorXd w_;
    Eigen::VectorXd de_;
    Eigen::MatrixXd J_;
    std::vector<char> constraint_signs_;
  } hiqp_constraints_;

  struct qpOASESProblem {
    qpOASESProblem(HiQPConstraints& hqp_constraints,
                   unsigned int solution_dims);

    ~qpOASESProblem();

    void setup();
    void solve();
    void getSolution(std::vector<double>& solution);

    qpOASES::MessageHandling mh;
    std::shared_ptr<qpOASES::QProblem> problem_;
    HiQPConstraints& hiqp_constraints_;
    unsigned int solution_dims_;

    std::vector<real_t> lb_dq_w_;  // lower bounds for dq
    std::vector<real_t> ub_dq_w_;  // upper bounds for dq

    std::vector<real_t> H_;   // Objective, positive semi-definite matrix
    std::vector<real_t> h_;   // Zero matrix in our case.
    std::vector<real_t> A_;   // Augmented Constraints
    std::vector<real_t> lb_A_; // Lower bound constraints (leq task)
    std::vector<real_t> ub_A_; // Upper bound (geq task)

    std::vector<real_t> dq_w_;

    qpOASES::int_t nWSR_; // No. of working set recalculations required.
    
  };

  unsigned int n_solution_dims_;  // number of solution dimensions
};
}
