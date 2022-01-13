/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"
#include "RobotController.h"
#include "SimulationBridge.h"
#include "main_helper.h"

MasterConfig gMasterConfig;

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage()
{
  printf("Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
         "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
         "\t      sim-or-robot: s for sim, r for robot\n"
         "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
         "                      this option can only be used in robot mode\n");
}

/*!
 * Setup and run the given robot controller
 */
int main_helper(int argc, char** argv, RobotController* ctrl)
{
  ros::init(argc, argv, "mit_mpc_node");

  if (argc != 3 && argc != 4)
  {
    printUsage();
    return EXIT_FAILURE;
  }

  gMasterConfig._robot = RobotType::MINI_CHEETAH;

  if (argv[2][0] == 's')
  {
    gMasterConfig.simulated = true;
  }
  else if (argv[2][0] == 'r')
  {
    gMasterConfig.simulated = false;
  }
  else
  {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argc == 4 && argv[3][0] == 'f')
  {
    gMasterConfig.load_from_file = true;
    printf("Load parameters from file\n");
  }
  else
  {
    gMasterConfig.load_from_file = false;
    printf("Load parameters from network\n");
  }

  printf("[Quadruped] Cheetah Software\n");
  printf("        Quadruped:  %s\n", gMasterConfig._robot == RobotType::MINI_CHEETAH ? "Mini Cheetah" : "Cheetah 3");
  printf("        Driver: %s\n", gMasterConfig.simulated ? "Development Simulation Driver" : "Quadruped Driver");

  // dispatch the appropriate driver
  if (gMasterConfig.simulated)
  {
    if (argc != 3)
    {
      printUsage();
      return EXIT_FAILURE;
    }
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH)
    {
      //using sim
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    }
    else
    {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  }
  else
  {
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH)
    {
//      uint8_t unitree_control_level = UNITREE_LEGGED_SDK::LOWLEVEL;

//      unitree_control_level = UNITREE_LEGGED_SDK::LOWLEVEL;

//      UNITREE_LEGGED_SDK::LCM roslcm(unitree_control_level);

      //using hw
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    }
    else
    {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  }

  return 0;
}
