#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <fstream>
#include <iostream>
#include <ostream>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/fwd.hpp>


#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/spatial-axis.hpp>

#include <ur_rtde/robotiq_gripper.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <filesystem>
#include <memory>

//#define SIMULATION true
#define SIMULATION false
#define NJOINTS 6
#define PINOCCHIO_ONLY false

int main(void) {
  // load robot model from urdf (most common type of robot description file)
  const std::string urdf_filename =
      std::string("../../robot_descriptions/urdf/ur5e_with_robotiq_hande.urdf");
  std::filesystem::path urdf_filename_absolute =
      std::filesystem::absolute(urdf_filename);
  const std::string mesh_dir =
      std::string("../../robot_descriptions/meshes/ur5e/");
  std::filesystem::path mesh_dir_absolute = std::filesystem::absolute(mesh_dir);
  // make that a pinocchio model ( this one is for kinematics and dynamics )
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename_absolute, model);
  // pinocchio separates dynamic data like joint angles into a different class
  // simply because it's better code design
  pinocchio::Data data(model);

  // setting up the visualizer, gepetto in this case.
  // pinocchio supports more of them,
  // and of course various simulators can be plugged in here with some
  // additional similar-looking boilerplate code
  pinocchio::GeometryModel visual_model;
  pinocchio::urdf::buildGeom(model, urdf_filename,
                             pinocchio::GeometryType::VISUAL, visual_model,
                             mesh_dir_absolute);
  pinocchio::GeometryModel collision_model;
  pinocchio::urdf::buildGeom(model, urdf_filename,
                             pinocchio::GeometryType::COLLISION,
                             collision_model, mesh_dir_absolute);

  // need to make these pointers so that i can have the object
  // without calling the constructor.
  // unique pointer as there will be only one instance of these
  std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control;
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;
  // TODO finish adding the gripper
  std::unique_ptr<ur_rtde::RobotiqGripper> gripper;
  // the constructor exists if it can not connect
  // the ip address is fixed on the robot (and has to be done on the client-side
  // as well) so there's no need to find it otherwise
  // TODO fix for pinocchio only runs
  if (!SIMULATION) {
    rtde_control =
        std::make_unique<ur_rtde::RTDEControlInterface>("192.168.1.102");
    rtde_receive =
        std::make_unique<ur_rtde::RTDEReceiveInterface>("192.168.1.102");
    gripper =
        std::make_unique<ur_rtde::RobotiqGripper>("192.168.1.102", 63352, true);
    gripper->connect();
	std::cout << "gripper connected" <<std::endl;
  } else {
    rtde_control = std::make_unique<ur_rtde::RTDEControlInterface>("127.0.0.1");
    rtde_receive = std::make_unique<ur_rtde::RTDEReceiveInterface>("127.0.0.1");
  }

  // the urdf treats the gripper
  // as 2 joints, but since the gripper does not change the position
  // of the arm, we pad q to 8
  Eigen::VectorXd q(model.nq);
  std::vector<double> qs_from_message;
  qs_from_message.reserve(8);
  float gripper_current_position;
  if (!PINOCCHIO_ONLY) {
    qs_from_message = rtde_receive->getActualQ();
    if (SIMULATION) {
      qs_from_message.push_back(0.0);
      qs_from_message.push_back(0.0);
    } else {
      gripper_current_position = gripper->getCurrentPosition();
	  std::cout << "read position" <<std::endl;
      qs_from_message.push_back((double)gripper_current_position);
      qs_from_message.push_back((double)gripper_current_position);
    }
    q = Eigen::Map<Eigen::VectorXd>(&qs_from_message[0], model.nq);
    qs_from_message.clear();
  } else {
    q = pinocchio::neutral(model);
  }

  // Mgoal is the desired pose ( position + orientation ).
  // making the rotation matrix 0s means we don't care about orientation
  // ( but this just means we keep the current one due to the rest of the math)
  const pinocchio::SE3 Mgoal(Eigen::Matrix3d::Zero(),
                             Eigen::Vector3d(0.5, 0.3, 0.5));
  const double eps = 1e-4;
  const int IT_MAX = 10000000;
  const double DT = 1e-3;
  const double damp = 1e-6;

  // define manipulator jacobian
  // it's a m x n matrix, where n=6 in 3D (3 linear, 3 angular dimensions for
  // velocity)
  // pinocchio::Data::Matrix6x J(model.nq, model.nv);
  pinocchio::Data::Matrix6x J(NJOINTS, model.nv);
  J.setZero();

  bool success = false;
  Eigen::Matrix<double, 6, 1> err_vector;
  Eigen::VectorXd v(model.nv);
  std::vector<double> v_as_std_vec;
  const int JOINT_ID = 6;
  for (int i = 0;; i++) {
    if (!PINOCCHIO_ONLY) {
      qs_from_message = rtde_receive->getActualQ();
      if (SIMULATION) {
        qs_from_message.push_back(0.0);
        qs_from_message.push_back(0.0);
      } else {
		// NOTE this is even used atm, it's just for checking the connection
        gripper_current_position = gripper->getCurrentPosition();
        qs_from_message.push_back((double)gripper_current_position);
        qs_from_message.push_back((double)gripper_current_position);
      }
      q = Eigen::Map<Eigen::VectorXd>(&qs_from_message[0], model.nq);
      qs_from_message.clear();
    }
    pinocchio::forwardKinematics(model, data, q);
    // data.omi[joint_id] is the placement of joint_id in the body frame
    // NOTE: in pinocchio, everything is expressed in the body frame by default
    // se3.act is the group action,
    // se3.actinv is the inverse group action
    // with this we get the action (rigid transform) from the current pose to
    // the goal pose
    const pinocchio::SE3 SEerror = data.oMi[JOINT_ID].actInv(Mgoal);
    // now we turn that into a tangent vector (in the joint frame) via the log
    // map
    err_vector = pinocchio::log6(SEerror).toVector();
    if (err_vector.norm() < eps) {
      success = true;
      std::cout << "reached destionation" << std::endl;
      break;
    }
    if (i >= IT_MAX) {
      success = false;
      std::cout << "FAIL: did not succed in IT_MAX iterations" << std::endl;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(SEerror.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    // TODO check if this actually changes the JJt diagonal (probably not)
    JJt.diagonal().array() += damp;
    // using cholesky to get the inverse applied to err jointly,
    // which is better complexity- and numerics-wise
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err_vector);
    //    std::cout << v << std::endl;
    //    we're running it on the real system
    if (!PINOCCHIO_ONLY) {
      // TODO make this assignment work
      //      v_as_std_vec.assign(v.data(), v.data() + v.size());
      v_as_std_vec.push_back(v[0]);
      v_as_std_vec.push_back(v[1]);
      v_as_std_vec.push_back(v[2]);
      v_as_std_vec.push_back(v[3]);
      v_as_std_vec.push_back(v[4]);
      v_as_std_vec.push_back(v[5]);
      rtde_control->speedJ(v_as_std_vec, 0.1, 1.0 / 500);
      v_as_std_vec.clear();
    } else {
      q = pinocchio::integrate(model, q, v * DT);
    }
    // TODO wrap this in an appropriate if
    viewer.display(q);
    if (!(i % 10000)) {
      std::cout << i << ": error = " << err_vector.transpose() << std::endl;
      std::cout << "pos" << data.oMi[JOINT_ID] << std::endl;
    }
  }

  if (success) {
    std::cout << "Convergence achieved!" << std::endl;
  } else {
    std::cout << "\nWarning: the iterative algorithm has not reached "
                 "convergence to the desired precision"
              << std::endl;
  }

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err_vector.transpose() << std::endl;

  return 0;
}
