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

#include <iomanip>

#include <kdl/frames.hpp>

namespace hiqp
{

  // Auxiliary function to 'ostream& << KDL::Tree&'
  void printChildrenToOstream(std::ostream& os, 
                const std::vector<KDL::SegmentMap::const_iterator>& children,
                std::vector<bool>& is_last_child,
                unsigned int level = 0) {
    is_last_child.push_back(false);
    for (auto&& child : children) {
      for (int i=0; i<level; ++i)
        os << (is_last_child[i] ? "   " : " | ");

      os << " + " << child->first 
      << " : " << child->second.segment.getJoint().getName() 
      << "(" << child->second.q_nr << ")"
      << "\n";

      is_last_child.at(level) = (child == children.back());
      printChildrenToOstream(
        os, 
        child->second.children, 
        is_last_child,
        level+1
        );
    }

    return;
  }

  std::ostream& operator<<(std::ostream& os, const KDL::Tree& kdl_tree) {
    KDL::SegmentMap::const_iterator root_segment = kdl_tree.getRootSegment();
    os << "nr of joints: " << kdl_tree.getNrOfJoints() << "\n";
    os << "nr of segments: " << kdl_tree.getNrOfSegments() << "\n";
    os << root_segment->first << "\n";

    std::vector<bool> is_last_child;
    printChildrenToOstream(os, root_segment->second.children, is_last_child);

    return os;
  }

  std::ostream& operator<<(std::ostream& os, const KDL::Vector& kdl_vector) {
    os << "[" << kdl_vector(0) << ", "
              << kdl_vector(1) << ", "
              << kdl_vector(2) << "]";
    return os;
  }

  std::ostream& operator<<(std::ostream& os, const KDL::FrameVel& kdl_frame_vel) {
    const KDL::Rotation &rot = kdl_frame_vel.value().M;
    const KDL::Vector &pos = kdl_frame_vel.value().p;
    const KDL::Vector &rotvel = kdl_frame_vel.deriv().rot;
    const KDL::Vector &vel = kdl_frame_vel.deriv().vel;

    os << std::setprecision(4) 
    << rot.data[0] << "  " << rot.data[1] << "  " << rot.data[2] << "     " 
    << pos.data[0] << "     " << rotvel.data[0] << "  " << vel.data[0] << "\n"

    << rot.data[3] << "  " << rot.data[4] << "  " << rot.data[5] << "     " 
    << pos.data[1] << "     " << rotvel.data[1] << "  " << vel.data[1] << "\n"

    << rot.data[6] << "  " << rot.data[7] << "  " << rot.data[8] << "     " 
    << pos.data[2] << "     " << rotvel.data[2] << "  " << vel.data[2];

    return os;
  }

  std::ostream& operator<<(std::ostream& os, const KDL::JntArrayVel& kdl_joints_vel) {
    const KDL::JntArray& q = kdl_joints_vel.q;
    const KDL::JntArray& qdot = kdl_joints_vel.qdot;

    unsigned int rows = q.rows();

    for (int i=0; i<rows; ++i) {
      os << std::setprecision(4) << "(" << q(i) << ", " << qdot(i) << ")\n";
    }

    return os;
  }

  std::ostream& operator<<(std::ostream& os, const KDL::Chain& kdl_chain) {
    unsigned int n_segments = kdl_chain.getNrOfSegments();
    os << n_segments << ": ";
    for (unsigned int i=0; i<n_segments; ++i) {
      const KDL::Segment& seg = kdl_chain.getSegment(i);
      os << "'" << seg.getName() << (i != n_segments-1 ? "'->" : "'");
    }

    return os;
  }

  int kdl_getAllQNrFromTree(const KDL::Tree& kdl_tree, std::vector<unsigned int>& qnrs) {
    qnrs.clear();
    for (auto&& element : kdl_tree.getSegments()) {
      qnrs.push_back(element.second.q_nr);
    }
  }

  std::string kdl_getJointNameFromQNr(const KDL::Tree& kdl_tree, unsigned int q_nr) {
    for (auto&& it : kdl_tree.getSegments()) {
      if (it.second.q_nr == q_nr) {
        return it.second.segment.getName();
      }
    }
    return "";
  }

  int kdl_getQNrFromJointName(const KDL::Tree& kdl_tree, const std::string& joint_name) {
    for (auto&& it : kdl_tree.getSegments()) {
      if (it.second.segment.getJoint().getName().compare(joint_name) == 0) {
        return it.second.q_nr;
      }
    }

    return -1;
  }

  int kdl_getQNrFromLinkName(const KDL::Tree& kdl_tree, const std::string& link_name) {
    KDL::SegmentMap::const_iterator it = kdl_tree.getSegments().find(link_name);
    if (it != kdl_tree.getSegments().end())
      return it->second.q_nr;
    else
      return -1;
  }

  // NOTE: This is a modified version of KDL::TreeJntToJacSolver::JntToJac!
  // This version supports giving the qdot as well.
  int kdl_JntToJac(const KDL::Tree& tree,
                   const KDL::JntArrayVel& qqdot, 
                   KDL::Jacobian& jac, 
                   const std::string& segmentname) {
    const KDL::JntArray& q_in = qqdot.q;
    const KDL::JntArray& qdot_in = qqdot.qdot;

    if (q_in.rows() != tree.getNrOfJoints() || 
      jac.columns() != tree.getNrOfJoints()) {
      return -1;
    }

    KDL::SegmentMap::const_iterator it = tree.getSegments().find(segmentname);

    if (it == tree.getSegments().end())
      return -2;

    jac.data.setZero();

    KDL::SegmentMap::const_iterator root = tree.getRootSegment();

    KDL::Frame T_total = KDL::Frame::Identity();

    while (it != root) {
           //get the corresponding q_nr for this TreeElement:
      unsigned int q_nr = GetTreeElementQNr(it->second);

           //get the pose of the segment:
      KDL::Frame T_local = 
        GetTreeElementSegment(it->second).pose(q_in(q_nr));

      T_total = T_local * T_total;

           //get the twist of the segment:
      if (GetTreeElementSegment(it->second).getJoint().getType() 
        != KDL::Joint::None) 
      {
        KDL::Twist t_local = 
          GetTreeElementSegment(it->second).twist(q_in(q_nr),
                              0);

        t_local = t_local.RefPoint(T_total.p - T_local.p);

        t_local = T_total.M.Inverse(t_local);

        jac.setColumn(q_nr,t_local);
      }

      it = GetTreeElementParent(it->second);
    }
       //Change the base of the complete jacobian from the endpoint to the base
    KDL::changeBase(jac, T_total.M, jac);

    return 0;
  }

  void printHiqpInfo(const std::string& msg) {
      std::cout << "[HiQP INFO] : " << msg << "\n";
  }

  void printHiqpWarning(const std::string& msg) {
      std::cerr << "[HiQP WARNING] : " << msg << "\n";
  }

} // namespace hiqp