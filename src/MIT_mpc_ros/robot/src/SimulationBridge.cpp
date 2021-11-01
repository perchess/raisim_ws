/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include "SimulationBridge.h"
#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"
#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"

/*!
 * Connect to a simulation
 */
void SimulationBridge::run() {
  // init shared memory:
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();

  install_segfault_handler(_sharedMemory().robotToSim.errorMessage);

  // init Quadruped Controller

  try {
    printf("[Simulation Driver] Starting main loop...\n");
    bool firstRun = true;
    for (;;) {
      // wait for our turn to access the shared memory
      // on the first loop, this gives the simulator a chance to put stuff in
      // shared memory before we start
      _sharedMemory().waitForSimulator();

      if (firstRun) {
        firstRun = false;
        // check that the robot type is correct:
        if (_robot != _sharedMemory().simToRobot.robotType) {
          printf(
            "simulator and simulatorDriver don't agree on which robot we are "
            "simulating (robot %d, sim %d)\n",
            (int)_robot, (int)_sharedMemory().simToRobot.robotType);
          throw std::runtime_error("robot mismatch!");
        }
      }

      // the simulator tells us which mode to run in
      _simMode = _sharedMemory().simToRobot.mode;
      switch (_simMode) {
        case SimulatorMode::RUN_CONTROL_PARAMETERS:  // there is a new control
          // parameter request
          handleControlParameters();
          break;
        case SimulatorMode::RUN_CONTROLLER:  // the simulator is ready for the
          // next robot controller run
          _iterations++;
          runRobotControl();
          break;
        case SimulatorMode::DO_NOTHING:  // the simulator is just checking to see
          // if we are alive yet
          break;
        case SimulatorMode::EXIT:  // the simulator is done with us
          printf("[Simulation Driver] Transitioned to exit mode\n");
          return;
          break;
        default:
          throw std::runtime_error("unknown simulator mode");
      }

      // tell the simulator we are done
      _sharedMemory().robotIsDone();
    }
  } catch (std::exception& e) {
    strncpy(_sharedMemory().robotToSim.errorMessage, e.what(), sizeof(_sharedMemory().robotToSim.errorMessage));
    _sharedMemory().robotToSim.errorMessage[sizeof(_sharedMemory().robotToSim.errorMessage) - 1] = '\0';
    throw e;
  }

}

/*!
 * This function handles a a control parameter message from the simulator
 */
void SimulationBridge::handleControlParameters() {
  ControlParameterRequest& request =
      _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response =
      _sharedMemory().robotToSim.controlParameterResponse;
  if (request.requestNumber <= response.requestNumber) {
    // nothing to do!
    printf(
        "[SimulationBridge] Warning: the simulator has run a ControlParameter "
        "iteration, but there is no new request!\n");
    return;
  }

  // sanity check
  u64 nRequests = request.requestNumber - response.requestNumber;
  assert(nRequests == 1);

  response.nParameters = _robotParams.collection._map
                             .size();  // todo don't do this every single time?

  switch (request.requestKind) {
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // do the actual set
      param.set(request.value, request.parameterKind);

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      printf("%s\n", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      std::string name(request.name);
      if(!_userParams) {
        printf("[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n");
      } else {
        ControlParameter& param = _userParams->collection.lookup(name);

        // type check
        if (param._kind != request.parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(request.parameterKind));
        }

        // do the actual set
        param.set(request.value, request.parameterKind);
      }

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      printf("%s\n", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // respond
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;  // acknowledge
      response.parameterKind =
          request.parameterKind;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind =
          request.requestKind;  // just for debugging print statements

      printf("%s\n", response.toString().c_str());
    } break;
    default:
      throw std::runtime_error("unhandled get/set");
  }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl() {
  if (_firstControllerRun) {
    printf("[Simulator Driver] First run of robot controller...\n");
    if (_robotParams.isFullyInitialized()) {
      printf("\tAll %ld control parameters are initialized\n",
             _robotParams.collection._map.size());
    } else {
      printf(
          "\tbut not all control parameters were initialized. Missing:\n%s\n",
          _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error(
          "not all parameters initialized when going into RUN_CONTROLLER");
    }

    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
    if(userControlParameters) {
      if (userControlParameters->isFullyInitialized()) {
        printf("\tAll %ld user parameters are initialized\n",
               userControlParameters->collection._map.size());
        _simMode = SimulatorMode::RUN_CONTROLLER;
      } else {
        printf(
            "\tbut not all control parameters were initialized. Missing:\n%s\n",
            userControlParameters->generateUnitializedList().c_str());
        throw std::runtime_error(
            "not all parameters initialized when going into RUN_CONTROLLER");
      }
    } else {
      _simMode = SimulatorMode::RUN_CONTROLLER;
    }


    _robotRunner->driverCommand =
        &_sharedMemory().simToRobot.gamepadCommand;
//    _robotRunner->spiData = &_sharedMemory().simToRobot.spiData;
    // Сюда вставить структуру spiData, сконструированную из топика ROS
//    _robotRunner->spiData = spiDataRos_;
//    spiDataRos_->fla
//    for (int leg = 0; leg < 4; leg++)
//    {
//      spiDataRos_->flags[leg] = _sharedMemory().simToRobot.spiData.flags[leg];
//    }
//    spiDataRos_->spi_driver_status = _sharedMemory().simToRobot.spiData.spi_driver_status;
//    spiDataRos_->
//    std::memcpy(spiDataRos_, &_sharedMemory().simToRobot.spiData, sizeof(SpiData));
    spiDataRos_ = &_sharedMemory().simToRobot.spiData;
    _robotRunner->spiData = spiDataRos_;
    _robotRunner->tiBoardData = _sharedMemory().simToRobot.tiBoardData;
    _robotRunner->robotType = _robot;
    _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
    _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;
    _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;
    _robotRunner->tiBoardCommand =
        _sharedMemory().robotToSim.tiBoardCommand;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData =
        &_sharedMemory().robotToSim.visualizationData;
    _robotRunner->cheetahMainVisualization =
        &_sharedMemory().robotToSim.mainCheetahVisualization;

    _robotRunner->init();
    _firstControllerRun = false;

    sbus_thread = new std::thread(&SimulationBridge::run_sbus, this);
  }
  _robotRunner->run();
}

/*!
 * Run the RC receive thread
 */
void SimulationBridge::run_sbus() {
  printf("[run_sbus] starting...\n");
  int port = init_sbus(true);  // Simulation
  while (true) {
    if (port > 0) {
      int x = receive_sbus(port);
      if (x) {
        sbus_packet_complete();
      }
    }
    usleep(5000);
  }
}

void SimulationBridge::raisimCallack(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  static bool init = true;
  if (init)
  {
    spiDataRos_ = new SpiData;
    _robotRunner->spiData = spiDataRos_;
//    std::memcpy(spiDataRos_, &_sharedMemory().simToRobot.spiData, sizeof(SpiData));
    init = false;
  }
  if (_robotRunner->spiData == &_sharedMemory().simToRobot.spiData)
    ROS_INFO("Get feddback from MIT");
  else if (_robotRunner->spiData == spiDataRos_)
  {
    ROS_INFO("Get feddback from RAISIM");
  }

  for (int leg = 0; leg < 4; leg++) {
    spiDataRos_->q_abad[leg] = msg->points.at(0).positions.at(leg * 3 + 0);
    spiDataRos_->q_hip[leg] = msg->points.at(0).positions.at(leg * 3 + 1);
    spiDataRos_->q_knee[leg] = msg->points.at(0).positions.at(leg * 3 + 2);

    spiDataRos_->qd_abad[leg] = msg->points.at(0).velocities.at(leg * 3 + 0);
    spiDataRos_->qd_hip[leg] = msg->points.at(0).velocities.at(leg * 3 + 1);
    spiDataRos_->qd_knee[leg] = msg->points.at(0).velocities.at(leg * 3 + 2);
  }


}
