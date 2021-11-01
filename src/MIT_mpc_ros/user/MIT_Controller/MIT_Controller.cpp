#include "MIT_Controller.hpp"


MIT_Controller::MIT_Controller():RobotController()
{
    pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/joint_group_position_controller/command", 100);
}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
    // Initialize a new GaitScheduler object
    _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

    // Initialize a new ContactEstimator object
    //_contactEstimator = new ContactEstimator<double>();
    ////_contactEstimator->initialize();

    // Initializes the Control FSM with all the required data
    _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                        _legController, _gaitScheduler,
                                        _desiredStateCommand, _controlParameters,
                                        _visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {
    // Find the current gait schedule
    _gaitScheduler->step();

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands();

    // Run the Control FSM code
    _controlFSM->runFSM();

    ROS_INFO_ONCE("Inside runController");

    if (_controlFSM->data._legController &&
        _controlFSM->currentState->stateName != FSM_StateName::PASSIVE &&
        _controlFSM->currentState->stateName != FSM_StateName::STAND_UP)
    {
        trajectory_msgs::JointTrajectory msg;
        static uint32_t seq = 0;
        msg.header.seq = seq++;

        static std::vector<std::string> joint_names = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
        msg.joint_names = joint_names;
        std::vector<double> q_vec = {
            _controlFSM->data._legController->commands[0].qDes(0),
            _controlFSM->data._legController->commands[0].qDes(1),
            _controlFSM->data._legController->commands[0].qDes(2),
            _controlFSM->data._legController->commands[1].qDes(0),
            _controlFSM->data._legController->commands[1].qDes(1),
            _controlFSM->data._legController->commands[1].qDes(2),
            _controlFSM->data._legController->commands[2].qDes(0),
            _controlFSM->data._legController->commands[2].qDes(1),
            _controlFSM->data._legController->commands[2].qDes(2),
            _controlFSM->data._legController->commands[3].qDes(0),
            _controlFSM->data._legController->commands[3].qDes(1),
            _controlFSM->data._legController->commands[3].qDes(2)
        };
        std::vector<double> v_vec = {
            _controlFSM->data._legController->commands[0].qdDes(0),
            _controlFSM->data._legController->commands[0].qdDes(1),
            _controlFSM->data._legController->commands[0].qdDes(2),
            _controlFSM->data._legController->commands[1].qdDes(0),
            _controlFSM->data._legController->commands[1].qdDes(1),
            _controlFSM->data._legController->commands[1].qdDes(2),
            _controlFSM->data._legController->commands[2].qdDes(0),
            _controlFSM->data._legController->commands[2].qdDes(1),
            _controlFSM->data._legController->commands[2].qdDes(2),
            _controlFSM->data._legController->commands[3].qdDes(0),
            _controlFSM->data._legController->commands[3].qdDes(1),
            _controlFSM->data._legController->commands[3].qdDes(2)
        };
        msg.points.resize(1);
        msg.points.at(0).positions = q_vec;
        msg.points.at(0).velocities = v_vec;
        msg.points.at(0).time_from_start.nsec = 1428571;
        pub_.publish(msg);
    }
    ros::spinOnce();

}


