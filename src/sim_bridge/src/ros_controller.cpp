#include <sim_bridge/ros_controller.h>


MPCControllerRos::MPCControllerRos(double freq)
  : step_freq_(freq)
  , world_()
  , scanDim1_(20)
  , scanDim2_(20)
  , buffer_size_(2000)
  , scans_counter_(0)
  , good_pts_{std::make_shared<boost::circular_buffer<Eigen::Vector3d>>(10000)}
  , visuals_buffer_(buffer_size_/2)
  , scans_pts_buffer_{std::make_shared<boost::circular_buffer<Eigen::Vector3d>>(buffer_size_)}
  //  , nh_("~")
{
  readParam<std::string>("~urdf_path", urdf_path_, "/home/den/catkin_workspaces/raisim_common/raisim_ros/src/a1_description/urdf/a1.urdf");
  raisimSetup();
  prev_q_ = Eigen::VectorXd::Zero(robot_->getGeneralizedCoordinateDim());
  prev_qd_ = VectorXd::Zero(robot_->getDOF());
    // cartesian_kp  cartesian_kd  joint_kp  joint_kd
  controller_ = new GaitCtrller(freq, std::vector<float>({100.0f, 1.0f, 0.01f, 0.05f}));// сделать рос параметрами
  generalizedFrorce_ = Eigen::VectorXd::Zero(robot_->getDOF());
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MPCControllerRos::cmdVelCallback,this);
  srv_mode_server_ = nh_.advertiseService(ros::this_node::getName() + "/set_robot_mode", &MPCControllerRos::srvSetMode, this);
  srv_gait_server_ = nh_.advertiseService(ros::this_node::getName() + "/set_gait_type", &MPCControllerRos::srvSetGait, this);
  df_callback_type = boost::bind(&MPCControllerRos::dynamicReconfigureCallback, this, _1, _2);
  df_server.setCallback(df_callback_type);
  effort_pub_ = nh_.advertise<unitree_legged_msgs::LowState>("effort_raisim",1);
  controller_->getConvexMpcPtr()->setPointsBuffer(good_pts_);
}

MPCControllerRos::~MPCControllerRos()
{
  std::cout<<"mass "<<robot_->getMassMatrix()[0]<<std::endl;
  raisim_server_->killServer();
}

void MPCControllerRos::raisimSetup()
{
  auto binaryPath = raisim::Path("/home/den/.raisim");
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
  // create raisim world
  world_.setTimeStep(1.0/step_freq_);
  world_.addGround();
  auto poddon1 = world_.addBox(1.0, 2.0, 0.15,0);
  poddon1->setName("poddon1");
  poddon1->setBodyType(raisim::BodyType::STATIC);
  poddon1->setPosition(0.0,0.0,0.0);
  poddon1->setOrientation(1.0,0.0,0.0,0.0);
  auto poddon2 = world_.addBox(1.0, 2.0, 0.15,0);
  poddon2->setName("poddon");
  poddon2->setBodyType(raisim::BodyType::STATIC);
  poddon2->setPosition(0.0,2.0 + 0.15,0.0);
  poddon2->setOrientation(1.0,0.0,0.0,0.0);
  // COM [0.3, -0.24375, 0.0] [-1.1003e-16, 2.38132, -0.0620116]
  // inertia moments [18.3001, 28.8333, 13.8001] [1.16388e+20, 8.33528e+19, 4.95214e+19] [116388, 83352.8, 49521.4]
  // [-2.50722e-16, 1.24683e-16, -2.15625]
//  raisim::Mat<3, 3> inertia({8.37, 0.0, 0.0,
//                             0.0, 9.1, -0.8358,
//                             0.0, -0.8358, 3.04});
//  raisim::Vec<3> com({0.0, 0.34, -0.18});
//  raisim::Mat<3, 3> inertia_;
//  inertia_.setIdentity();
//  const raisim::Vec<3> com_ = {0, 0, 0};
//  auto stairs = world_.addMesh("/home/den/catkin_workspaces/raisim_common/raisim_ros/src/raisim_ros/rsc/world/stairs/solidworks/solid_stairs.obj", 1000.0, inertia, com_,0.0005);
//  stairs->setPosition(0.0,3.0,0.0);
//  stairs->setOrientation(0.707,0.707,0.0,0.0);
  robot_ = world_.addArticulatedSystem(urdf_path_);
  Eigen::VectorXd jointNominalConfig(robot_->getGeneralizedCoordinateDim());

  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
      0.06, 0.6, -1.2,
      -0.06, 0.6, -1.2,
      0.06, 0.6, -1.2,
      -0.06, 0.6, -1.2;

//  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
//      -0.23110604286193848, 0.7660617828369141, -1.930681824684143,
//      0.2086973786354065, 0.7694507837295532, -1.945541501045227,
//      -0.2868724763393402, 0.7470470666885376, -1.8848075866699219,
//      0.2648811340332031, 0.7518696784973145, -1.9018760919570923;
  //  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
  //      -0.23110604286193848, -0.7660617828369141, 1.930681824684143,
  //      0.2086973786354065, -0.7694507837295532, 1.945541501045227,
  //      -0.2868724763393402, -0.7470470666885376, 1.8848075866699219,
  //      0.2648811340332031, -0.7518696784973145, 1.9018760919570923;
  robot_->setGeneralizedCoordinate(jointNominalConfig);



  robot_->setGeneralizedForce(Eigen::VectorXd::Zero(robot_->getDOF()));

  robot_->setName("minicheetah");
  /// launch raisim server
  raisim_server_ = new raisim::RaisimServer(&world_);
  raisim_server_->launchServer();
  raisim_server_->focusOn(robot_);

  /// Setup laser dimensions

//  for(int i=0; i<scanDim1_; i++)
//    for(int j=0; j<scanDim2_; j++)
//      scans_.push_back(raisim_server_->addVisualSphere("sphere" + std::to_string(i) + "/" + std::to_string(j), 0.01, 1, 0, 0));
  for (size_t i = 0; i < visuals_buffer_.capacity(); i++)
  {
    visuals_buffer_.push_back(raisim_server_->addVisualSphere("sphere" + std::to_string(i), 0.01, 1, 0, 0));
    visuals_buffer_.back()->setPosition(0,0,0);
  }

  // Вспомогательные маркеры
  raisim_server_->addVisualSphere("pfinal", 0.05, 1, 0, 0);
  raisim_server_->getVisualObject("pfinal")->setColor(1,1,0,1);
//  raisim_server_->addVisualSphere("pstart", 0.05, 1, 0, 0);
//  raisim_server_->getVisualObject("pstart")->setColor(0,1,0,1);
//  raisim_server_->addVisualSphere("pcur", 0.05, 1, 0, 0);
//  raisim_server_->getVisualObject("pcur")->setColor(1,1,1,1);
//  raisim_server_->addVisualSphere("foot_FR_ground_truth", 0.05, 1, 0, 0);
//  raisim_server_->getVisualObject("foot_FR_ground_truth")->setColor(1,1,.5,1);
//  raisim_server_->addVisualSphere("pf_FR_est", 0.05, 1, 0, 0);
//  raisim_server_->getVisualObject("pf_FR_est")->setColor(1,0,0,1);
}

void MPCControllerRos::preWork()
{
  Eigen::VectorXd stand_pos(19);
  stand_pos << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
      0.06, 0.6, -1.2,
      -0.06, 0.6, -1.2,
      0.06, 0.6, -1.2,
      -0.06, 0.6, -1.2;

  Eigen::VectorXd stand_vel(Eigen::VectorXd::Zero(18));
  Eigen::VectorXd jointPgain(robot_->getDOF());
  Eigen::VectorXd jointDgain(robot_->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(10.0);
  robot_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  size_t iters = 4000;
  // Задержка чтобы успел прогрузиться мир
  for (size_t i = 0; i < iters; i++)
  {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        robot_->setPdTarget(stand_pos, stand_vel);
        robot_->setPdGains(jointPgain, jointDgain);
        raisim_server_->integrateWorldThreadSafe();
  }
  robot_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  iters = 10;
  controller_->SetRobotMode(0);
  controller_->SetGaitType(STANDING);
  for (size_t i = 0; i < iters; i++)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(long(world_.getTimeStep() * 1000000)));
    raisim_server_->integrateWorldThreadSafe();
    robot_->getState(q_, qd_);
    a1_feedback(q_, qd_);
    updateFeedback();
    controller_->PreWork(imu_, legdata_);
    prev_q_ = q_;
    prev_qd_ = qd_;
  }
  // Запускаем обработчик лидара
    timer_ = nh_.createTimer(ros::Duration(0.25), &MPCControllerRos::depthSensorWork, this);
}

void MPCControllerRos::updateFeedback()
{
  tf::Matrix3x3 rot_mat(tf::Quaternion(q_(4), q_(5),q_(6), q_(3)));
  Eigen::VectorXd acc = (qd_ - prev_qd_) * step_freq_;
  acc(2) += 9.8;
  imu_.accelerometer(0, 0) = rot_mat[0][0] * acc(0) + rot_mat[1][0]*acc(1) + rot_mat[2][0] * acc(2);
  imu_.accelerometer(1, 0) = rot_mat[0][1] * acc(0) + rot_mat[1][1]*acc(1) + rot_mat[2][1] * acc(2);
  imu_.accelerometer(2, 0) = rot_mat[0][2] * acc(0) + rot_mat[1][2]*acc(1) + rot_mat[2][2] * acc(2);
  imu_.quat(0, 0) = q_(4);
  imu_.quat(1, 0) = q_(5);
  imu_.quat(2, 0) = q_(6);
  imu_.quat(3, 0) = q_(3);
  imu_.gyro(0, 0) = rot_mat[0][0] * qd_(3) + rot_mat[1][0]*qd_(4)+ rot_mat[2][0] * qd_(5);
  imu_.gyro(1, 0) = rot_mat[0][1] * qd_(3) + rot_mat[1][1]*qd_(4)+ rot_mat[2][1] * qd_(5);
  imu_.gyro(2, 0) = rot_mat[0][2] * qd_(3) + rot_mat[1][2]*qd_(4)+ rot_mat[2][2] * qd_(5);

  std::vector<float> vec_q(q_.data() + 7, q_.data() + q_.size());
  std::vector<float> vec_qd(qd_.data() + 6, qd_.data() + qd_.size());
  legdata_ = LegData(vec_q, vec_qd);
}

void MPCControllerRos::spin()
{
//  robot_->setBasePos(raisim::Vec<3>({0,0,0.8}));
//  robot_->setBaseOrientation_e(Eigen::Matrix3d::Identity(3,3));
  ros::spinOnce();
  std::this_thread::sleep_for(std::chrono::microseconds(long(world_.getTimeStep() * 1000000)));
  raisim_server_->integrateWorldThreadSafe();
//  depthSensorWork(); ушло в таймер
  robot_->getState(q_, qd_);
  a1_feedback(q_, qd_);
  updateFeedback();
  effort_ = controller_->TorqueCalculator(imu_, legdata_);
  a1_effort(effort_);
  publishEffort_toRos(effort_);
  generalizedFrorce_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_;
  robot_->setGeneralizedForce(generalizedFrorce_);
  prev_q_ = q_;
  prev_qd_ = qd_;
  drawVisual();
}

void MPCControllerRos::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  twist_ = *msg;
  controller_->SetRobotVel(twist_.linear.x, twist_.linear.y, twist_.angular.z);
}

bool MPCControllerRos::srvSetMode(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                                  quadruped_ctrl::QuadrupedCmdBoolResponse &res)
{
  controller_->SetRobotMode(req.cmd);
  res.result = true;
  res.description = "Service to set robot mode. 1 for low energy mode, 0 for high perfomance mode.";
  return true;
}

bool MPCControllerRos::srvSetGait(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                                  quadruped_ctrl::QuadrupedCmdBoolResponse &res)
{
  if (0 <= req.cmd && req.cmd < 12)
  {
    controller_->SetGaitType(req.cmd);
    res.result = true;
  }
  else if (req.cmd == 13)// Прыжок
  {
    controller_->jump(true);
    res.result = true;
  }
  else
    res.result = false;
  switch  (req.cmd)
  {
  case TROT:
    res.description = "Set TROT";
    break;
  case BUNDING:
    res.description = "Set BUNDING";
    break;
  case PRONKING:
    res.description = "Set PRONKING";
    break;
  case STANDING:
    res.description = "Set STANDING";
    break;
  case TROT_RUN:
    res.description = "Set TROT_RUN";
    break;
  case GALLOPING:
    res.description = "Set GALLOPING";
    break;
  case PACING:
    res.description = "Set PACING";
    break;
  case WALK1:
    res.description = "Set WALK1";
    break;
  case WALK2:
    res.description = "Set WALK2";
    break;
  default:
    res.description = "Default behavior";
  }
  return true;
}

//  Изменения в направлениях осей для unitree a1
void a1_effort (Eigen::VectorXd& eff)
{
  float effthreshold = 50.0;
  for (size_t joint = 0; joint < 12; joint++)
  {
  eff(joint) = eff(joint) >= effthreshold ? effthreshold : eff(joint);
  eff(joint) = eff(joint) <= -effthreshold ? -effthreshold : eff(joint);
  }
  for (int i = 0; i < 4; i++)
  {
//    eff(i * 3) = -eff(i * 3);
    eff(i * 3 + 1) = -eff(i * 3 + 1);
    eff(i * 3 + 2) = -eff(i * 3 + 2);
  }
}

//  Изменения в направлениях осей для unitree a1
void a1_feedback(Eigen::VectorXd& q, Eigen::VectorXd& qd)
{
  for (int i = 0; i < 4; i++)
  {
//    q(i*3+7) = -q(i*3+7);
    q(i * 3 + 1+7) = -q(i * 3 + 1+7);
    q(i * 3 + 2+7) = -q(i * 3 + 2+7);
//    qd(i*3+6) = -qd(i*3+6);
    qd(i * 3 + 1+6) = -qd(i * 3 + 1+6);
    qd(i * 3 + 2+6) = -qd(i * 3 + 2+6);
  }
}


void MPCControllerRos::dynamicReconfigureCallback(quadruped_msgs::generalConfig &config, uint32_t level){
  ROS_INFO("Get new config");
  // TODO: Передавать конфигу в контроллер, а там разбивать параметры
  controller_->updateConfig(config);
}

void MPCControllerRos::publishEffort_toRos(Eigen::VectorXd& effort)
{
  unitree_legged_msgs::LowState msg;
  msg.header.stamp = ros::Time::now();
  for (size_t joint = 0; joint < 12; joint++)
  {
    msg.motorState.at(joint).tauEst = effort(joint);
  }
  effort_pub_.publish(msg);
}

void MPCControllerRos::depthSensorWork(const ros::TimerEvent& event)
{
  raisim::Vec<3> lidarPos;
  raisim::Mat<3,3> lidarOri;
  Eigen::Vector3d direction;
  robot_->getFramePosition("depth_joint", lidarPos);
  robot_->getFrameOrientation("depth_joint", lidarOri);

  for(int i=0; i<scanDim1_; i++)
  {
    for (int j = 0; j < scanDim2_; j++)
    {
      const double yaw = j * M_PI / scanDim2_ * 0.6 - 0.3 * M_PI;
      double pitch = -(i * 1.0/scanDim1_) + 0.1;
      const double normInv = 1. / sqrt(pitch * pitch + 1);
      direction = {cos(yaw) * normInv, sin(yaw) * normInv, -pitch * normInv};
      Eigen::Vector3d rayDirection;
      rayDirection = lidarOri.e() * direction;
      auto &col = world_.rayTest(lidarPos.e(), rayDirection, 5);
      if (col.size() > 0)
      {
        if (canPlace(col[0].getPosition(), *scans_pts_buffer_))
        {
          scans_pts_buffer_->push_back(col[0].getPosition());
          visuals_buffer_.at(scans_counter_)->setPosition(col[0].getPosition());
        }
      }
      scans_counter_ = scans_counter_ < visuals_buffer_.capacity() - 1 ? scans_counter_ + 1 : 0;

//        scans_[i * scanDim2_ + j]->setPosition(col[0].getPosition());
      //      else
      //        scans_[i * scanDim2_ + j]->setPosition({0, 0, 100});
    }
  }

  /// Покрасить точки в "яме" в синий цвет, остальные в красный
  double avg = avgBufferPoints(scans_pts_buffer_);
  for (auto it:visuals_buffer_)
  {
    // Яма
    if (it->getPosition().z() < avg)
    {
      it->setColor(0,0,1,1);

    }
    // Поверхность
    else
    {
      it->setColor(1,0,0,1);
      if (canPlace(it->getPosition(), *good_pts_))
        good_pts_->push_back(it->getPosition());
    }
  }

}

void  MPCControllerRos::drawVisual()
{
  ///
  /// DRAW
  ///
  raisim::Vec<3> FR_foot_pose;
  robot_->getFramePosition("FR_foot_fixed", FR_foot_pose);

  Vec3<float> pf_FR = controller_->Pf_;
  Vec3<float> p0_FR = controller_->getConvexMpcPtr()->getFootTrajVect()[0].getStartPosition();
  Vec3<float> pcur_FR = controller_->getConvexMpcPtr()->getFootTrajVect()[0].getPosition();
  Vec3<float> step_len_FR = controller_->getConvexMpcPtr()->getFootTrajVect()[0].getStepLength();
  Vec3<double> pf_fr_estimated(FR_foot_pose.e().x() + step_len_FR.x(),
                               FR_foot_pose.e().y() + step_len_FR.y(),
                               good_pts_->front().z());

//  std::cout << "step len x: " << step_len_FR.x() <<  "step len y: " << step_len_FR.y() << std::endl;

  raisim_server_->getVisualObject("pfinal")->setPosition(pf_FR.x(),pf_FR.y(),pf_FR.z());
//  raisim_server_->getVisualObject("pstart")->setPosition(p0_FR.x(),p0_FR.y(),good_pts_->front().z());

//  raisim_server_->getVisualObject("pcur")->setPosition(pcur_FR.x(),pcur_FR.y(),good_pts_->front().z());

//  raisim_server_->getVisualObject("foot_FR_ground_truth")->setPosition(FR_foot_pose.e().x(),
//                                                          FR_foot_pose.e().y(),
//                                                          FR_foot_pose.e().z());

//  raisim_server_->getVisualObject("pf_FR_est")->setPosition(pf_fr_estimated.x(),
//                                                            pf_fr_estimated.y(),
//                                                            pf_fr_estimated.z());

  ///
  /// PROCESSING
  ///
  auto closest = findClosestPoint(pf_fr_estimated, good_pts_);
  auto dist = closest - pf_fr_estimated;
//  std::cout << "dist X : " << dist.x() << " dist Y : " << dist.y() << std::endl;
//  controller_->getConvexMpcPtr()->setPfCorrection(dist.x(), dist.y());


  //  double dist = calcMinDistance(pf_FR);
  //  static double eps = 0.5;
  //  ROS_INFO_STREAM("DIST " << dist);
  //  for (auto it:good_pts_)
  //  {
  //    // Тестовая проверка принадлежности Pf к точке "поддона"
  ////    double dist = calcDistance(controller_->getConvexMpcPtr()->getFootTrajVect()[0].getFinalPosition(),
  ////        it->getPosition());
  //    if (std::abs(dist) <= eps)
  //    {
  //      ROS_INFO_STREAM("GAP");
  //      ROS_INFO_STREAM( "dist " << std::abs(dist));
  //      ROS_INFO_STREAM( "pc point  " << it->getPosition());
  //      ROS_INFO_STREAM( "pf  " << controller_->getConvexMpcPtr()->getFootTrajVect()[0].getFinalPosition());
  //      break;;
  //    }
  //  }
}


double avgVector(std::vector<raisim::Visuals *> const& v) {
  return 1.0 * std::accumulate(v.begin(), v.end(), 0.0,
                               [&](double a, raisim::Visuals * b){return a + b->getPosition().z(); }) / v.size();
}


double avgBufferVisuals(boost::circular_buffer<raisim::Visuals *> const& v) {
  return 1.0 * std::accumulate(v.begin(), v.end(), 0.0,
                               [&](double a, raisim::Visuals * b){return a + b->getPosition().z(); }) / v.size();
}


double avgBufferPoints(std::shared_ptr<boost::circular_buffer<Eigen::Vector3d>> const& v) {
  return 1.0 * std::accumulate(v->begin(), v->end(), 0.0,
                               [&](double a, Eigen::Vector3d b){return a + b.z(); }) / v->size();
}

double calcDistance(Vec3<float> const& pf, Eigen::Vector3d const& point)
{
  return sqrt(pow(pf.x() - point.x(), 2) +
              pow(pf.y() - point.y(), 2) +
              pow(pf.z() - point.z(), 2)
              );
}


double calcDistance(Eigen::Vector3d const& p1, Eigen::Vector3d const& p2)
{
  return sqrt(pow(p1.x() - p2.x(), 2) +
              pow(p1.y() - p2.y(), 2) +
              pow(p1.z() - p2.z(), 2)  );
}

double MPCControllerRos::calcMinDistance(Vec3<float> const& pf)
{
  double dist = 9999;
  double cur_dist = 0;
  for (auto it:*good_pts_)
  {
    cur_dist = calcDistance(pf, it);
    if (cur_dist < dist)
      dist = cur_dist;
  }
  return dist;
}


const Eigen::Vector3d MPCControllerRos::findClosestPoint(const Eigen::Vector3d& point,
                                                         std::shared_ptr<boost::circular_buffer<Eigen::Vector3d>>& buffer)
{
  double dist = 9999;
  double cur_dist = 0;
  Eigen::Vector3d ans = buffer->front();
  for (auto it:*buffer)
  {
    cur_dist = calcDistance(point, it);
    if (cur_dist < dist)
    {
      dist = cur_dist;
      ans = it;
    }
  }
  return ans;
}


const Eigen::Vector3d MPCControllerRos::findClosestPoint(const Eigen::Vector3d& point,
                                                         boost::circular_buffer<Eigen::Vector3d>& buffer)
{
  double dist = 9999;
  double cur_dist = 0;
  Eigen::Vector3d ans = buffer.front();
  for (auto it:buffer)
  {
    cur_dist = calcDistance(point, it);
    if (cur_dist < dist)
    {
      dist = cur_dist;
      ans = it;
    }
  }
  return ans;
}

bool MPCControllerRos::canPlace(const Eigen::Vector3d& point,
                                boost::circular_buffer<Eigen::Vector3d>& buffer)
{
  static double thresh = 0.005; // метр
  auto closest = findClosestPoint(point, buffer);
//  std::cout << "distance = " << calcDistance(point, closest) << std::endl;
  if (calcDistance(point, closest) >= thresh)
    return true;
  return false;
}
