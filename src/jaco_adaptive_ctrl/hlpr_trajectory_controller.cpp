#include <jaco_adaptive_ctrl/jaco_trajectory_controller.h>

using namespace std;

JacoTrajectoryController::JacoTrajectoryController() : pnh("~"),
  smoothTrajectoryServer(pnh, "follow_joint_trajectory", boost::bind(&JacoTrajectoryController::executeSmoothTrajectory, this, _1), false)
{
  pnh.param("max_curvature", maxCurvature, 100.0);
  pnh.param("sim", sim_flag_, false);
  pnh.param<std::string>("joint_state_topic", jointStateTopic, "/j2s7s300_driver/out/joint_state");

  jointNames.clear();
  jointNames.push_back("j2s7s300_joint_1");
  jointNames.push_back("j2s7s300_joint_2");
  jointNames.push_back("j2s7s300_joint_3");
  jointNames.push_back("j2s7s300_joint_4");
  jointNames.push_back("j2s7s300_joint_5");
  jointNames.push_back("j2s7s300_joint_6");
  jointNames.push_back("j2s7s300_joint_7");

  joint_kp.clear();
  joint_kp.push_back(KP);
  joint_kp.push_back(KP);
  joint_kp.push_back(KP);
  joint_kp.push_back(KP);
  joint_kp.push_back(KP);
  joint_kp.push_back(KP);
  joint_kp.push_back(KP);

  joint_kv.clear();
  joint_kv.push_back(KV);
  joint_kv.push_back(KV);
  joint_kv.push_back(KV);
  joint_kv.push_back(KV);
  joint_kv.push_back(KV);
  joint_kv.push_back(KV);
  joint_kv.push_back(KV);

  jointStates.position.resize(NUM_JACO_JOINTS);
  jointStates.velocity.resize(NUM_JACO_JOINTS);
  jointStates.effort.resize(NUM_JACO_JOINTS);
  jointStates.name.resize(NUM_JACO_JOINTS);

  trajectoryPoint.joint1 = 0.0;
  trajectoryPoint.joint2 = 0.0;
  trajectoryPoint.joint3 = 0.0;
  trajectoryPoint.joint4 = 0.0;
  trajectoryPoint.joint5 = 0.0;
  trajectoryPoint.joint6 = 0.0;
  trajectoryPoint.joint7 = 0.0;

  // Setting up simulation vs. real robot
  if(!sim_flag_)
  {
    ROS_INFO("Using real robot arm.");

    // Connect to the low-level angular driver from kinova-ros
    angularCmdPublisher = n.advertise<kinova_msgs::JointVelocity>("j2s7s300_driver/in/joint_velocity", 1);
  }
  else
  {
    ROS_INFO("Using simulation robot arm.");

    // Setup a fake gravity comp service (torque control)
    start_gravity_comp_ = n.advertiseService(
                "j2s7s300_driver/in/start_gravity_comp", &JacoTrajectoryController::startGravityCompService, this);
    stop_gravity_comp_ = n.advertiseService(
                "j2s7s300_driver/in/stop_gravity_comp", &JacoTrajectoryController::stopGravityCompService, this);

    // Setup a fake admittance service (Force control)
    start_force_control_service_ = n.advertiseService("j2s7s300_driver/in/start_force_control", &JacoTrajectoryController::startForceControlCallback, this);
    stop_force_control_service_ = n.advertiseService("j2s7s300_driver/in/stop_force_control", &JacoTrajectoryController::stopForceControlCallback, this);

    // Connect to the gazebo low-level ros controller
    angCmdSimPublisher = n.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/command", 1);
  }

  // Subscribes to the joint states of the robot
  jointStatesSubscriber = n.subscribe(jointStateTopic.c_str(), 1, &JacoTrajectoryController::jointStateCallback, this);

  // Start the trajectory server
  smoothTrajectoryServer.start();
}

/** Fake Gravity Comp Services for Simulation **/
bool JacoTrajectoryController::startGravityCompService(kinova_msgs::Start::Request &req,
                                             kinova_msgs::Start::Response &res)
{
    ROS_INFO("Simulation 'enabled' grav comp. Note: nothing actually happens");
    res.start_result = "Start gravity compensation requested.";
    return true;
}

/** Fake Gravity Comp Services for Simulation **/
bool JacoTrajectoryController::stopGravityCompService(kinova_msgs::Stop::Request &req,
                                             kinova_msgs::Stop::Response &res)
{
    ROS_INFO("Simulation 'disabled' grav comp. Note: nothing actually happens");
    res.stop_result = "Stop gravity compensation requested.";
    return true;
}

bool JacoTrajectoryController::startForceControlCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res)
{
    ROS_INFO("Simulation 'enabled' admittance mode. Note: nothing actually happens");
    res.start_result = "Start force control requested.";
    return true;
}

bool JacoTrajectoryController::stopForceControlCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res)
{
    ROS_INFO("Simulation 'disabled' admittance mode. Note: nothing actually happens");
    res.stop_result = "Stop force control requested.";
    return true;
}

void JacoTrajectoryController::jointStateCallback(const sensor_msgs::JointState &msg)
{
  // Cycle through the number of JACO joints
  for (int joint_id = 0; joint_id < NUM_JACO_JOINTS; joint_id++){

    // Find the location of the joint
    string joint_name = jointNames[joint_id];
    int msg_loc = distance(msg.name.begin(), find(msg.name.begin(), msg.name.end(), joint_name));

    jointStates.position[joint_id] = msg.position[msg_loc];
    jointStates.name[joint_id] = msg.name[msg_loc];
    jointStates.velocity[joint_id] = msg.velocity[msg_loc];
    jointStates.effort[joint_id] = msg.effort[msg_loc];
  }

  joint_state_flag = true;
}

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

void JacoTrajectoryController::executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  int numPoints = goal->trajectory.points.size();
  int origNumPoints = goal->trajectory.points.size();
  // change to timing vector if there are too few points, see added start state for splining below
  // (have to change this up here because ecl::Array is not easy to dynamically change!)
  bool prepend_curr_pos = goal->trajectory.points[0].time_from_start.toSec() != 0;
  int correctedNumPoints = numPoints;
  if (prepend_curr_pos)
  {
    correctedNumPoints++;
  }

  vector< ecl::Array<double> > jointPoints;
  jointPoints.resize(NUM_JACO_JOINTS);
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    jointPoints[i].resize(correctedNumPoints);
  }

  
  ecl::Array<double> timePoints(correctedNumPoints);
  if (prepend_curr_pos)
  {
    timePoints[0] = 0;
  }

  int offset = correctedNumPoints - numPoints;

  // get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {
    bool includedJoints[NUM_JACO_JOINTS] = { };
    timePoints[i+offset] = goal->trajectory.points[i].time_from_start.toSec();

    for (unsigned int trajectoryIndex = 0; trajectoryIndex < goal->trajectory.joint_names.size(); trajectoryIndex++)
    {
      string jointName = goal->trajectory.joint_names[trajectoryIndex];
      int jointIndex = distance(jointNames.begin(), find(jointNames.begin(), jointNames.end(), jointName));
      if (jointIndex >= 0 && jointIndex < NUM_JACO_JOINTS)
      {
        jointPoints[jointIndex][i+offset] = goal->trajectory.points.at(i).positions.at(trajectoryIndex);
        includedJoints[jointIndex] = true;
      }
    }

    // Fill non-included joints with current joint state
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
      if (!includedJoints[j])
        jointPoints[j][i+offset] = jointStates.position[j];
  }

  // add start state if the trajectory has too few points to calculate a spline
  if (prepend_curr_pos)
  {
    ROS_WARN("WARNING: goal trajectory does not start at t=0, adding current robot state as trajectory start to allow for splining...");
    // this is a problem - treats jointPoints like it's timeXjoints, when it's actually jointsXtime
    ecl::Array<double> startPoints(NUM_JACO_JOINTS);
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      jointPoints[i][0] = jointStates.position[i];
    }
    numPoints ++;
  }

  // Gather timing corrections for trajectory segments that violate max velocity
//  float correctedTime[numPoints] = { };  // doesn't compile in indigo...
  float correctedTime[numPoints];
  for (unsigned int i = 0; i < numPoints; i ++)
  {
    correctedTime[i] = 0;
  }
//  ecl::Array<double> correctedTime(numPoints);
  for (unsigned int i = 1; i < numPoints; i++)
  {
    float maxTime = 0.0;
    float vel = 0.0;

    float plannedTime = timePoints[i] - timePoints[i-1];
    bool validTime = plannedTime > 0;

    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      float time = fabs(jointPoints[j][i] - jointPoints[j][i-1]);
      if (plannedTime > 0)
        vel = fabs(jointPoints[j][i] - jointPoints[j][i-1]) / plannedTime;

      if (j <= 3)
      {
        time /= 0.9*LARGE_ACTUATOR_VELOCITY;
        if (plannedTime > 0 && vel > 0.9*LARGE_ACTUATOR_VELOCITY) {
          ROS_WARN("Invalid velocity: %f on joint %d", vel, j);
          validTime = false;
        }
          
      }
      else
      {
        time /= 0.9*SMALL_ACTUATOR_VELOCITY;
        if (plannedTime > 0 && vel > 0.9*SMALL_ACTUATOR_VELOCITY){
          ROS_WARN("Invalid velocity: %f on joint %d", vel, j);
          validTime = false;
        }
      }

      if (time > maxTime)
        maxTime = time;
    }

    if (!validTime)
      correctedTime[i] = maxTime;
  }


  // Apply timing corrections
  for (unsigned int i = 1; i < numPoints; i++)
  {
    correctedTime[i] += correctedTime[i-1];
    timePoints[i] += correctedTime[i];
  }

  // Print warning if time corrections applied
  if (correctedTime[numPoints-1] > 0)
  {
    ROS_WARN("Warning: Timing of joint trajectory violates max velocities, using computed time");
    if (ros::service::exists("/move_group/trajectory_execution/set_parameters", false))
    {
      dynamic_reconfigure::ReconfigureRequest req;
      dynamic_reconfigure::ReconfigureResponse resp;

      ros::service::call("/move_group/trajectory_execution/set_parameters", req, resp);

      // Note: doesn't compile in indigo
//      for (auto const& it : resp.config.bools)
//        if (it.name == "execution_duration_monitoring" && it.value)
//          ROS_WARN("Warning: Execution duration monitoring turned on. This may cause trajectory to be premempted before completion.");
      for (unsigned int i = 0; i < resp.config.bools.size(); i ++)
      {
        if (resp.config.bools[i].name == "execution_duration_monitoring" && resp.config.bools[i].value)
        {
          ROS_WARN("Warning: Execution duration monitoring turned on. This may cause trajectory to be premempted before completion.");
        }
      }
    }
  }

  // Spline the given points to smooth the trajectory
  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(NUM_JACO_JOINTS);

  // Setup cubic storage in case
  vector<ecl::CubicSpline> cubic_splines;
  cubic_splines.resize(NUM_JACO_JOINTS);
  cubic_flag_=false;
  try
  {
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], maxCurvature);
      splines.at(i) = tempSpline;
    }
  }
  catch (...) // This catches ALL exceptions
  {
    
    cubic_flag_= true;
    ROS_WARN("WARNING: Performing cubic spline rather than smooth linear because of crash");
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      ecl::CubicSpline tempSpline = ecl::CubicSpline::Natural(timePoints, jointPoints[i]);
      cubic_splines.at(i) = tempSpline;
    }
  }

  //control loop
  bool trajectoryComplete = false;
  double startTime = ros::Time::now().toSec();
  double t = 0;
  float error[NUM_JACO_JOINTS];
  float totalError;
  float prevError[NUM_JACO_JOINTS] = {0};
  float currentPoint;
  double current_joint_pos[NUM_JACO_JOINTS];
  ros::Rate rate(1000);  // spinning rate, should be faster than the 100hz timer for publishing commands
  bool reachedFinalPoint;
  ros::Time finalPointTime;

  // Check if we send the trajectory to simulation vs. real robot
  // if sim just send position trajectory and not run the PID loop
  if (sim_flag_)
  {
    ROS_INFO("WARNING: Simulation velocity trajectory is executed as a position trajectory");

    // Setup msg JointTrajectory for std gazebo ros controller
    // Populate JointTrajectory with the points in the current goal
    trajectory_msgs::JointTrajectory jtm;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtm.joint_names = goal->trajectory.joint_names;
    jtm.points = goal->trajectory.points;

    // Publish out trajaectory points listed in the goal
    angCmdSimPublisher.publish(jtm);

    // Wait a second for the arm to move a bit
    ros::Duration(0.1).sleep();

    // Check the total error to determine if the trajectory finished
    totalError = 1.0;
    float prevError = 0.0;
    bool jointError = true;
    while (abs(totalError - prevError) > 0.001 && jointError)
    {
      prevError = totalError;

      // Copy from joint_state publisher to current joints
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        current_joint_pos[i] = jointStates.position[i];
      }
      // Compute total error of all joints away from last position
      totalError = 0;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle(goal->trajectory.points[origNumPoints - 1].positions[i]),currentPoint) - currentPoint;
        totalError += fabs(error[i]);
        jointError = jointError || error[i] > ERROR_THRESHOLD;
      }
      // Rate to check if error has changed
      ros::Duration(0.1).sleep();
    }

    // Tell the server we finished the trajectory
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);

  }
  else
  {
    // Sending to the real robot
    sensor_msgs::JointState armState;
    armState.position.resize(NUM_JACO_JOINTS);
    armState.velocity.resize(NUM_JACO_JOINTS);
    armState.effort.resize(NUM_JACO_JOINTS);
    armState.name.resize(NUM_JACO_JOINTS);
    joint_state_flag = true;

    ros::Timer cmd_publisher = n.createTimer(ros::Duration(0.01), &JacoTrajectoryController::publishCmd, this);
    while (!trajectoryComplete)
    {
      while (!joint_state_flag)
      {
        rate.sleep();
        ros::spinOnce();
      }

      //check for preempt requests from clients
      if (smoothTrajectoryServer.isPreemptRequested())
      {
        boost::mutex::scoped_lock lock(trajectory_point_mutex);

        cmd_publisher.stop();

        //stop gripper control
        trajectoryPoint.joint1 = 0.0;
        trajectoryPoint.joint2 = 0.0;
        trajectoryPoint.joint3 = 0.0;
        trajectoryPoint.joint4 = 0.0;
        trajectoryPoint.joint5 = 0.0;
        trajectoryPoint.joint6 = 0.0;
        trajectoryPoint.joint7 = 0.0;

        angularCmdPublisher.publish(trajectoryPoint);

        //preempt action server
        smoothTrajectoryServer.setPreempted();
        ROS_INFO("Smooth trajectory server preempted by client");

        return;
      }

      //get time for trajectory
      t = ros::Time::now().toSec() - startTime;
      if (t > timePoints.at(timePoints.size() - 1))
      {
        //use final trajectory point as the goal to calculate error until the error
        //is small enough to be considered successful
        if (!reachedFinalPoint)
        {
          reachedFinalPoint = true;
          finalPointTime = ros::Time::now();
        }

        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        bool jointError = false;
        double maxError = 0;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);

          // Check if we're using a cubic or a linear spline
          double splineValue;
          if (cubic_flag_)
          {
            splineValue = (cubic_splines.at(i))(timePoints.at(timePoints.size() - 1));
          }
          else
          {
            splineValue = (splines.at(i))(timePoints.at(timePoints.size() - 1));
          }
          // Now generate the value
          error[i] = nearest_equivalent(simplify_angle(splineValue),
                                        currentPoint) - currentPoint;
          jointError = jointError || fabs(error[i]) > ERROR_THRESHOLD;
        }

        if (!jointError || ros::Time::now().toSec() - finalPointTime.toSec() >= 5.0)
        {

          if (!jointError)
          {
            ROS_INFO("No joint error");
          }
          else if (ros::Time::now().toSec() - finalPointTime.toSec() >= 5.0)
          {
            ROS_INFO("5 second timeout reached");
          }

          cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;
          //stop arm
          {
            boost::mutex::scoped_lock lock(trajectory_point_mutex);
            cmd_publisher.stop();
            trajectoryPoint.joint1 = 0.0;
            trajectoryPoint.joint2 = 0.0;
            trajectoryPoint.joint3 = 0.0;
            trajectoryPoint.joint4 = 0.0;
            trajectoryPoint.joint5 = 0.0;
            trajectoryPoint.joint6 = 0.0;
            trajectoryPoint.joint7 = 0.0;
            angularCmdPublisher.publish(trajectoryPoint);
          }
          trajectoryComplete = true;
          ROS_INFO("Trajectory complete!");
          break;
        }
      }
      else
      {
        //calculate error
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          current_joint_pos[i] = jointStates.position[i];
        }

        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          // Check if we're using a cubic or a linear spline
          double splineValue;
          if (cubic_flag_)
          {
            splineValue = (cubic_splines.at(i))(t);
          }
          else
          {
            ecl::SmoothLinearSpline tmp_spline = splines.at(i);
            splineValue = tmp_spline(t);
          }
          error[i] = nearest_equivalent(simplify_angle(splineValue), currentPoint) - currentPoint;
        }
      }


      //calculate control input
      //populate the velocity command
      {
        boost::mutex::scoped_lock lock(trajectory_point_mutex);
        trajectoryPoint.joint1 = (float)(joint_kp[0] * error[0] + joint_kv[0] * (error[0] - prevError[0]));
        trajectoryPoint.joint2 = (float)(joint_kp[1] * error[1] + joint_kv[1] * (error[1] - prevError[1]));
        trajectoryPoint.joint3 = (float)(joint_kp[2] * error[2] + joint_kv[2] * (error[2] - prevError[2]));
        trajectoryPoint.joint4 = (float)(joint_kp[3] * error[3] + joint_kv[3] * (error[3] - prevError[3]));
        trajectoryPoint.joint5 = (float)(joint_kp[4] * error[4] + joint_kv[4] * (error[4] - prevError[4]));
        trajectoryPoint.joint6 = (float)(joint_kp[5] * error[5] + joint_kv[5] * (error[5] - prevError[5]));
        trajectoryPoint.joint7 = (float)(joint_kp[6] * error[6] + joint_kv[6] * (error[6] - prevError[6]));
      }

      //for debugging:
      // cout << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << ", " << error[6] << endl;

      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        prevError[i] = error[i];
      }

      rate.sleep();
      ros::spinOnce();

      joint_state_flag = false;
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smoothTrajectoryServer.setSucceeded(result);
  }
}

void JacoTrajectoryController::publishCmd(const ros::TimerEvent &t)
{
  boost::mutex::scoped_lock lock(trajectory_point_mutex);
  angularCmdPublisher.publish(trajectoryPoint);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_trajectory_controller");

  JacoTrajectoryController jtc;
  ros::spin();
}

