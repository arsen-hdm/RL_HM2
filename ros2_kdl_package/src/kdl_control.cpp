#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntrOther(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // Retrieve the current state of the manipulator
    KDL::Frame ee_frame = robot_->getEEFrame();                
    KDL::Twist ee_velocity = robot_->getEEVelocity();          
    Eigen::MatrixXd jacobian = robot_->getEEJacobian().data;   
    Eigen::VectorXd jacobian_dot_q_dot = robot_->getEEJacDotqDot();

    // Compute errors in operational space
    Vector6d e;    
    Vector6d edot; 
    computeErrors(_desPos, ee_frame, _desVel, ee_velocity, e, edot);

    // Compute gain matrices
    Eigen::VectorXd Kp(6), Kd(6);
    Kp << _Kpp, _Kpp, _Kpp, _Kpo, _Kpo, _Kpo;
    Kd << _Kdp, _Kdp, _Kdp, _Kdo, _Kdo, _Kdo;
    
    // Compute desired acceleration in operational space
    Eigen::VectorXd acc_des(6);
    acc_des.head(3) = toEigen(_desAcc.vel);
    acc_des.tail(3) = toEigen(_desAcc.rot);

    // Control output combining desired acceleration and PD terms
    Eigen::VectorXd control_output = acc_des + Kd.cwiseProduct(edot) + Kp.cwiseProduct(e);

    Eigen::VectorXd q_ddot = pseudoinverse(jacobian) * (control_output - jacobian_dot_q_dot);

    // Compute control law
    Eigen::MatrixXd Jsim = robot_->getJsim();                
    Eigen::VectorXd n = robot_->getCoriolis();
    Eigen::VectorXd tau = Jsim * q_ddot + n;

    return tau;
}

