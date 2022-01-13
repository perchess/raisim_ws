/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/StateEstimatorContainer.h"
#include "SimUtilities/IMUTypes.h"
#include "rt/rt_rc_interface.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "JPosInitializer.h"
#include <ros/ros.h>

#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "Utilities/PeriodicTask.h"
#include "cheetah_visualization_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "RobotController.h"
#include <lcm-cpp.hpp>
#include <unitree_legged_msgs/LowCmd.h>

class RobotRunner : public PeriodicTask
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController*, PeriodicTaskManager*, float, std::string);
  using PeriodicTask::PeriodicTask;
  void init() override;
  void run() override;
  void cleanup() override;

  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator(bool cheaterMode = false);
  virtual ~RobotRunner();

  RobotController* _robot_ctrl;

  GamepadCommand* driverCommand;
  RobotType robotType;
  VectorNavData* vectorNavData;
  CheaterState<double>* cheaterState;
  SpiData* spiData;
  SpiCommand* spiCommand;
  TiBoardCommand* tiBoardCommand;
  TiBoardData* tiBoardData;
  RobotControlParameters* controlParameters;
  VisualizationData* visualizationData;
  CheetahVisualization* cheetahMainVisualization;

 private:
  float _ini_yaw;

  int iter = 0;

  void setupStep();
  void finalizeStep();

  JPosInitializer<float>* _jpos_initializer;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  bool _cheaterModeEnabled = false;
  DesiredStateCommand<float>* _desiredStateCommand;
  rc_control_settings rc_control;
  lcm::LCM _lcm;
  leg_control_command_lcmt leg_control_command_lcm;
  state_estimator_lcmt state_estimator_lcm;
  leg_control_data_lcmt leg_control_data_lcm;
  // Contact Estimator to calculate estimated forces and contacts

//  unitree_legged_msgs::LowCmd low_cmd;

  const unsigned char _HIGHLEVEL = 0x00;
  const unsigned char _LOWLEVEL  = 0xff;

  const double _PosStopF = (2.146E+9f);
  const double _VelStopF = (16000.0f);

//  float Kp = 1;
//  float Kd = 1;

//  struct Joint
//  {
//    float q = {0};
//    float dq = {0};
//    float q_d = {0};
//    float q_init = {0};
//    float dq_d = {0};
//    float tau = {0};
//  } _joint[12];

//  struct Body
//  {
//    float gyro[3] = {0};
//    float acc[3] = {0};
//    float quat[4] = {0};
//  }_body;

  ros::NodeHandle _n;
  ros::Subscriber _sub_low_state;
  ros::Publisher _pub_low_cmd;

  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
};

#endif  // PROJECT_ROBOTRUNNER_H
