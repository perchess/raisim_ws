#pragma once
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <raisim/object/ArticulatedSystem/ArticulatedSystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <GaitCtrller.h>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <quadruped_ctrl/QuadrupedCmdBool.h>
#include <dynamic_reconfigure/server.h>
#include <quadruped_msgs/generalConfig.h>
#include <boost/circular_buffer.hpp>

enum Gaits
{
  TROT = 0,
  BUNDING = 1,
  PRONKING = 2,
  STANDING = 4,
  TROT_RUN = 5,
  GALLOPING = 7,
  PACING = 8,
  WALK1 = 10,
  WALK2 = 11
};


class MPCControllerRos
{
public:
  MPCControllerRos(double freq);
  ~MPCControllerRos();

  void raisimSetup();
  void preWork();
  void readRosParams();
  void updateFeedback();
  void spin();
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
  bool srvSetMode(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                  quadruped_ctrl::QuadrupedCmdBoolResponse &res);

  bool srvSetGait(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                  quadruped_ctrl::QuadrupedCmdBoolResponse &res);
  //
  void dynamicReconfigureCallback(quadruped_msgs::generalConfig &config, uint32_t level);
  void publishEffort_toRos(Eigen::VectorXd& effort);
  void depthSensorWork(const ros::TimerEvent& event);
  void drawVisual();
  double calcMinDistance(Vec3<float> const& pf);
  const Eigen::Vector3d findClosestPoint(const Eigen::Vector3d& point, boost::circular_buffer<Eigen::Vector3d>& buffer);
  // Стоит ли добавлять точку в буфер?
  bool canPlace(const Eigen::Vector3d& point);

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::ServiceServer srv_mode_server_;
  ros::ServiceServer srv_gait_server_;
  ros::Timer timer_;
  geometry_msgs::Twist twist_;
  GaitCtrller* controller_;
  double step_freq_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd prev_q_;
  Eigen::VectorXd prev_qd_;

//  std::vector<float> pid_params_;
  raisim::ArticulatedSystem* robot_;
  raisim::RaisimServer* raisim_server_;
  raisim::World world_;
  VectorNavData imu_;
  LegData legdata_;
  Eigen::VectorXd effort_;
  Eigen::VectorXd generalizedFrorce_;
  std::string urdf_path_;
  dynamic_reconfigure::Server<quadruped_msgs::generalConfig> df_server;
  dynamic_reconfigure::Server<quadruped_msgs::generalConfig>::CallbackType df_callback_type;
  ros::Publisher effort_pub_;
  std::vector<raisim::Visuals *> scans_;
  int scanDim1_;
  int scanDim2_;
  size_t buffer_size_;
  uint16_t scans_counter_;
  boost::circular_buffer<Eigen::Vector3d> good_pts_;
  boost::circular_buffer<raisim::Visuals *> visuals_buffer_;
  boost::circular_buffer<Eigen::Vector3d> scans_pts_buffer_;
};

//! @brief Шаблоннная функция для чтения параметров
template <typename T>
void readParam(const std::string param_name, T& param_value,
               const T default_value) {
  if (!ros::param::get(param_name, param_value)) {
    ROS_WARN_STREAM("Parameter \""
                    << param_name << "\" didn' find in Parameter Server."
                    << "\nSetting default value: " << &default_value);
    param_value = default_value;
  }
}

//  Изменения в направлениях осей для unitree a1
void a1_effort (Eigen::VectorXd& eff);


//  Изменения в направлениях осей для unitree a1
void a1_feedback(Eigen::VectorXd& q, Eigen::VectorXd& qd);

// Расчет среднего значения в векторе по z-составляющей
double avgVector(std::vector<raisim::Visuals *> const& v);

double avgBufferVisuals(boost::circular_buffer<raisim::Visuals *> const& v);

double avgBufferPoints(boost::circular_buffer<Eigen::Vector3d> const& v);

double calcDistance(Vec3<float> const& pf, Eigen::Vector3d const&);

