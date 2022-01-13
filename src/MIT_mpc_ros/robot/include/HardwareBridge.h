/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#ifdef linux

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <lord_imu/LordImu.h>

#include <lcm-cpp.hpp>
#include <string>

#include <ros/ros.h>
//#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>
//#include "convert.h"

#define RATE 1000 //rate of sleep loop

/*!
 * Interface between robot and hardware
 */
class HardwareBridge
{
public:
  HardwareBridge(RobotController* robot_ctrl)
    : statusTask(&taskManager, 0.5f), _interfaceLCM(getLcmUrl(255)), _visualizationLCM(getLcmUrl(255))//, _safety(UNITREE_LEGGED_SDK::LeggedType::A1)
  {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();

    _sub_low_state = _n.subscribe("/low_state", 1, &HardwareBridge::_lowStateCallback, this);

    _pub_low_cmd = _n.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);

  }
  void prefaultStack();
  void setupScheduler();
  void initError(const char* reason, bool printErrno = false);
  void initCommon();
  ~HardwareBridge()
  {
    delete _robotRunner;
  }
  void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const gamepad_lcmt* msg);

  void handleInterfaceLCM();
  void handleControlParameter(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                              const control_parameter_request_lcmt* msg);

  void publishVisualizationLCM();
  void run_sbus();

protected:
  PeriodicTaskManager taskManager;
  PrintTaskStatus statusTask;
  GamepadCommand _gamepadCommand;
  VisualizationData _visualizationData;
  CheetahVisualization _mainCheetahVisualization;
  lcm::LCM _interfaceLCM;
  lcm::LCM _visualizationLCM;
  //UNITREE_LEGGED_SDK::LCM roslcm(unitree_control_level);
  control_parameter_respones_lcmt _parameter_response_lcmt;
  SpiData _spiData;
  SpiCommand _spiCommand;

  TiBoardCommand _tiBoardCommand[4];
  TiBoardData _tiBoardData[4];

  bool _firstRun = true;
  RobotRunner* _robotRunner = nullptr;
  RobotControlParameters _robotParams;
  u64 _iterations = 0;
  std::thread _interfaceLcmThread;
  volatile bool _interfaceLcmQuit = false;
  RobotController* _controller = nullptr;
  ControlParameters* _userControlParameters = nullptr;

  ros::NodeHandle _n;
  ros::Subscriber _sub_low_state;
  ros::Publisher _pub_low_cmd;

  void _lowStateCallback(unitree_legged_msgs::LowState msg);

//  const unsigned char _HIGHLEVEL = 0x00;
  const unsigned char _LOWLEVEL  = 0xff;

  const double _PosStopF = (2.146E+9f);
  const double _VelStopF = (16000.0f);

//  float Kp = 1;
//  float Kd = 1;

  struct Joint
  {
    float q = {0};
    float dq = {0};
    float q_d = {0};
    float q_init = {0};
    float dq_d = {0};
    float tau = {0};
  } _joint[12];

  struct Body
  {
    float gyro[3] = {0};
    float acc[3] = {0};
    float quat[4] = {0};
  }_body;

  int _port;

//  UNITREE_LEGGED_SDK::LowCmd _send_low_lcm = {0};
//  UNITREE_LEGGED_SDK::LowState _recv_low_lcm = {0};
//  unitree_legged_msgs::LowCmd _send_low_ros;
//  unitree_legged_msgs::LowState _recv_low_ros;

//  UNITREE_LEGGED_SDK::HighCmd _send_high_lcm = {0};
//  UNITREE_LEGGED_SDK::HighState _recv_high_lcm = {0};
//  unitree_legged_msgs::HighCmd _send_high_ros;
//  unitree_legged_msgs::HighState _recv_high_ros;

//  UNITREE_LEGGED_SDK::Safety _safety;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 */
class MiniCheetahHardwareBridge : public HardwareBridge
{
public:
  MiniCheetahHardwareBridge(RobotController* rc, bool load_parameters_from_file);
  void runSpi();
  void initHardware();
  void run();
  void runMicrostrain();
  void logMicrostrain();
  void abort(const std::string& reason);
  void abort(const char* reason);

private:
  VectorNavData _vectorNavData;
  lcm::LCM _spiLcm;
  lcm::LCM _microstrainLcm;
  std::thread _microstrainThread;
  LordImu _microstrainImu;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;
};

#endif  // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
