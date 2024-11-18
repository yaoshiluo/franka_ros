#include <cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>

namespace my_impedance_controller {
    bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                            ros::NodeHandle& node_handle) {
        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        sub_equilibrium_pose_ = node_handle::subscribe(
            "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
            ros::TransportHints().reliable().tcpNoDelay());

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR_STREAM("CartesianImpedanceController: could not read parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR("CartesianImpedanceController: Invalid or no joint_names parameters provided, aborting controller init!");
            return false;
        } 

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("CartesianImpedanceController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("CartesianImpedanceController: Expection getting model handle from interface: " << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("CartesianImpedanceController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i =0; i < 7; ++i) {
            try{
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                EOS_ERROR_STREAM("CartesianImpedanceController: Expection getting joint handles: " << ex.what())
                return false;
            }
        }

        dynamic_reconfifure_compliance_param_node_ = 
            ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

        dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<my_impedance_controller::compliance_paramConfig>>(
            dynamic_reconfigure_compliance_param_node_); 

        dynamic_server_compliance_param_->setCallback(
            boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        cartesian_stiffness_.setZero();
        cartesian_damping_.setZero();
                         
        return ture;                   
    }

    void CartesianImpedanceController::starting(const ros::Time&) {
        // 获取机器人当前的状态信息, O_T_EE: 世界坐标系到末端执行器的变换矩阵
        franka::RobotState initial_state = state_handle_->getRobotState();
        // 计算末端执行器在当前姿态下的雅可比矩阵, franka::Frame::kEndEffector 表示计算末端执行器相对于基坐标系的雅可比矩阵, 
        //雅可比矩阵维度为 6 x 7（6表示线速度和角速度，7表示机器人关节数）
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

        Eigen::Map<Eigen::Matrix<double, 7,1>> q_initial(initial_state.q.data());
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaternion orientation(transform.rotation());

        Eigen::Matrix<double, 6, 1>error;
        error.head(3) << position - position_d_;

        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }

        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

        // 姿态误差需转换到base frame,位置误差不需要
        error.tail(3) << -transform.rotation() * error.tail(3);

        Eigen::VectorXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        //Cartesian PD control
        tau_task << jacobian.transpose() * 
                        (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));

        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - 
                            jacobian.transpose() * jacobian_transpose_pinv) *
                            (nullspace_stiffness_ * (q_d_nullspace_ -q) -
                            (2.0 * sqrt(nullspace_stiffness_)) * dq);

        //Desired torque
        tau_d << tau_task + tau_nullspace + coriolis;
        //Saturate torque rate to avoid discontinuities
        tau_d << saturateTorqueRate(tau_d, tau_J_d);
        
        //将计算得到的目标力矩 tau_d 应用于每个机器人关节上
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        // update parameters changed online either through dynamic reconfigure or through the interactive
        // target by filtering
        cartesian_stiffness_ =
            filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
        cartesian_damping_ =
            filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
        nullspace_stiffness_ =
            filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
        std::lock_guard<std::mutex> position_d_target_mutex_lock(
            position_and_orientation_d_target_mutex_);
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);        
    }

    Eigen::Matrix<double, 7, 1>CartesianImpedanceController::saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d) {
        Eigen::Matrix<double, 7, 1>tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = 
                tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }

        return tau_d_saturated;
        }

    
}