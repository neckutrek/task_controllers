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

#include <hiqp/utilities.h>
#include <hiqp/solvers/gurobi_solver.h>

#include <gurobi_c++.h>
#include <ros/assert.h>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#define OUTPUT_FLAG      0
#define PRESOLVE         -1
#define OPTIMALITY_TOL   1e-6
#define SCALE_FLAG       1
#define TIME_LIMIT       1.0//0.005
#define DUAL_REDUCTIONS  1
#define TIKHONOV_FACTOR  5*1e-5

namespace hiqp
{

  GurobiSolver::GurobiSolver() {
    env_.set(GRB_IntParam_OutputFlag, OUTPUT_FLAG);
    env_.set(GRB_IntParam_Presolve, PRESOLVE);
    env_.set(GRB_DoubleParam_OptimalityTol, OPTIMALITY_TOL);
    env_.set(GRB_IntParam_ScaleFlag, SCALE_FLAG);
    env_.set(GRB_DoubleParam_TimeLimit, TIME_LIMIT);
    env_.set(GRB_IntParam_DualReductions, DUAL_REDUCTIONS);
  }

  /// \bug Empty stages are ignored, and the lower ones gain a hierarchy rank 
  bool GurobiSolver::solve(std::vector<double>& solution) {
    if (stages_map_.empty())
      return false;

    reset();

    unsigned int x_dim = solution.size();
    A_.resize(Eigen::NoChange, x_dim);
    StageMap::iterator it = stages_map_.begin();
    try {
      // Iterate over all stages and solve one QP per each stage
      unsigned int s_count = 0; // stage counter

      for (it; it!=stages_map_.end(); ++it) {
	//DEBUG ================================================
	// std::cerr<<"Stage priority: "<<it->first<<std::endl;
	// std::cerr<<"Stage Jacobian: "<<std::endl<<it->second.J_<<std::endl;
	// std::cerr<<"Stage de*: "<<std::endl<<it->second.e_dot_star_.transpose()<<std::endl;
	//DEBUG END ================================================

        assert(it->second.J_.cols() == x_dim);
        s_count++;
        GRBModel model(env_);
        unsigned int s_dim = it->second.nRows; //dimension of the current stage
        unsigned int s_acc_dim = b_.rows(); //accumulated dimensions of all the previously solved stages

        // append the new signs, jacobian matrix and task velocity vector to the previous ones
        assert(it->second.constraint_signs_.size()==s_dim);
        for(unsigned int i = 0; i<s_dim; i++) {
          if (it->second.constraint_signs_.at(i)== 0)
            senses_.push_back(GRB_EQUAL);
          else if (it->second.constraint_signs_.at(i)== -1)
            senses_.push_back(GRB_LESS_EQUAL);
          else if (it->second.constraint_signs_.at(i)== 1)
            senses_.push_back(GRB_GREATER_EQUAL);
          else {
            ROS_ERROR("Invalid sign specification in HQP stage %d!",s_count);
            return false;
          }
        }
        b_.conservativeResize(s_acc_dim + s_dim);
        b_.tail(s_dim) = it->second.e_dot_star_;
        A_.conservativeResize(s_acc_dim + s_dim, Eigen::NoChange);
        A_.bottomRows(s_dim) = it->second.J_;
        w_.conservativeResize(s_acc_dim + s_dim);
        w_.tail(s_dim).setZero(); //set zero for now - will be filled with the solution later

        //at each iteration, variables are x (the joint velocities) + the slack variables
        double* lb_x = new double[x_dim]; std::fill_n(lb_x, x_dim, -GRB_INFINITY);
        double* ub_x = new double[x_dim]; std::fill_n(ub_x, x_dim, GRB_INFINITY);
        GRBVar* x =model.addVars(lb_x, ub_x, NULL, NULL, NULL, x_dim);

        double* lb_w = new double[s_dim]; std::fill_n(lb_w, s_dim, -GRB_INFINITY);
        double* ub_w = new double[s_dim]; std::fill_n(ub_w, s_dim, GRB_INFINITY);
        GRBVar* w =model.addVars(lb_w, ub_w, NULL, NULL, NULL, s_dim); //slack variables
        model.update();

        GRBLinExpr* lhsides = new GRBLinExpr[s_dim + s_acc_dim];
        // char* senses = new char[s_dim + s_acc_dim];
        double* rhsides = new double[s_dim + s_acc_dim];
        double* coeff_x = new double[x_dim];
        double* coeff_w = new double[s_dim];
        // ========== CREATE GUROBI EXPRESSIONS ==========

        //Fill in the constant right-hand side array
        Eigen::Map<Eigen::VectorXd>(rhsides, s_acc_dim + s_dim) = b_ + w_;

        //Fill in the left-hand side expressions
        for(unsigned int i=0; i<s_acc_dim; i++) {
          Eigen::Map<Eigen::VectorXd>(coeff_x, x_dim) = A_.row(i);
          lhsides[i].addTerms(coeff_x, x, x_dim);
        }
        for(unsigned int i=0; i<s_dim; i++) {
          Eigen::Map<Eigen::VectorXd>(coeff_x, x_dim) = A_.row(s_acc_dim+i);
          lhsides[s_acc_dim+i].addTerms(coeff_x, x, x_dim);
          if(s_count == 1)
            lhsides[s_acc_dim+i] -= w[i]*0.0; //force the slack variables to be zero in the highest stage;
          else
            lhsides[s_acc_dim+i] -= w[i];
        }

        //add constraints
        GRBConstr* constrs=model.addConstrs(lhsides,&senses_[0],rhsides,NULL,s_acc_dim + s_dim);

        //add objective
        GRBQuadExpr obj;
        std::fill_n(coeff_x, x_dim, TIKHONOV_FACTOR);
        std::fill_n(coeff_w, s_dim, 1.0);
        obj.addTerms(coeff_x, x, x, x_dim);
        obj.addTerms(coeff_w, w, w, s_dim);
        model.setObjective(obj, GRB_MINIMIZE);
        model.update();

        // DEBUG =============================================
        // std::cerr<<std::setprecision(2)<<"Gurobi solver stage "<<s_count<<" matrices:"<<std::endl;
	// std::cerr<<"A"<<std::endl<<A_<<std::endl;
	// std::cerr<<"signs: ";
        // for (unsigned k=0; k<senses_.size(); k++)
	//   std::cerr<<senses_[k]<<" ";

	// std::cerr<<std::endl<<"b"<<b_.transpose()<<std::endl;
	// std::cerr<<"w"<<w_.transpose()<<std::endl;
	// DEBUG END ==========================================

        // ========== SOLVE ==========
        model.optimize();
        int status = model.get(GRB_IntAttr_Status);
        double runtime = model.get(GRB_DoubleAttr_Runtime);

        if (status != GRB_OPTIMAL) {
          if(status == GRB_TIME_LIMIT)
            ROS_WARN("Stage solving runtime %f sec exceeds the set time limit of %f sec.", runtime, TIME_LIMIT);
          else
            ROS_ERROR("In HQPSolver::solve(...): No optimal solution found for stage %d. Status is %d.", s_count, status);

          delete[] lb_x;
          delete[] ub_x;
          delete[] lb_w;
          delete[] ub_w;
          delete[] x;
          delete[] w;
          delete[] lhsides;
          // delete[] senses;
          delete[] rhsides;
          delete[] coeff_x;
          delete[] coeff_w;
            
	  //model.write("/home/rkg/Desktop/model.lp");
	  //model.write("/home/yumi/Desktop/model.sol");

          return false;
        }

        try {
          //Update the current solution
          for(unsigned int i=0; i<x_dim; i++)
            solution.at(i) = x[i].get(GRB_DoubleAttr_X);
          for(unsigned int i=0; i<s_dim; i++)
            w_(s_acc_dim + i) = w[i].get(GRB_DoubleAttr_X);
        } catch(GRBException e) {
          std::cerr<<"In HQPSolver::solve(...): Gurobi exception with error code" <<e.getErrorCode()<<", and error message "<<e.getMessage().c_str()<<" when trying to extract the solution variables."<<std::endl;
          // model.write("/home/yumi/Desktop/model.lp");
          // model.write("/home/yumi/Desktop/model.sol");

          return false;
        }

        delete[] lb_x;
        delete[] ub_x;
        delete[] lb_w;
        delete[] ub_w;
        delete[] x;
        delete[] w;
        delete[] lhsides;
        // delete[] senses;
        delete[] rhsides;
        delete[] coeff_x;
        delete[] coeff_w;
      } // for (it; it!=stages_map_.end(); ++it)
    } catch(GRBException e) {
      std::cerr<<"In HQPSolver::solve(...): Gurobi exception with error code "<<e.getErrorCode()<<", and error message "<<e.getMessage().c_str()<<"."<<std::endl;
      return false;
    }
    return true;
  }

  void GurobiSolver::reset() {
    b_.resize(0);
    w_.resize(0);
    A_.resize(0,0);
    senses_.clear();
  }

} // namespace hiqp
