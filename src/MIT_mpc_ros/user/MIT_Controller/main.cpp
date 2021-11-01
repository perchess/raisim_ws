/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "MIT_Controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mit_controller_node");
  main_helper(argc, argv, new MIT_Controller());
  return 0;
}
