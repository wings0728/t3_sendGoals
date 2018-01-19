#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sstream>
#include <vector>
#include <string>
//signal
#include <signal.h>
//geometry
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <math.h>

#include "t3_description/goal.h"
//boost
// #include <boost/bind.hpp>
// #include <boost/thread/mutex.hpp>
// #include <boost/shared_ptr.hpp>

#define kMinDistance 0.1

//----------------------------------------------------------------------------------------------------------chengyuen3/1
#define roundTrip 1
#define durationSlp 2.0
//----------------------------------------------------------------------------------------------------------

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalsNode
{
public:
	GoalsNode();
	~GoalsNode();
	void process();

private:
//----------------------------------------------------------------------------------------------------------chengyuen3/1
	MoveBaseClient ac;
//----------------------------------------------------------------------------------------------------------
	//frame_id
  bool _isMapGoal;
	std::string _frameId;
	int _countOfGoals;
	int _currentIdxOfGoal;
	bool _arriveGoal;
	//all poses of goals
  std::vector<std::pair<double, std::pair<double, double> > > _posesOfGoals;//the goals from param-file
  std::pair<double, std::pair<double, double> > _poseOfGoal;//the goal from qt-map clicked
	//std::vector<double> _posesOfGoals;
	//tf::Pose _posesOfGoals;
	ros::NodeHandle _nh;
	ros::NodeHandle _privateNh;
	move_base_msgs::MoveBaseGoal _currentGoal;
	move_base_msgs::MoveBaseGoal _lastGoal;
	// ros::Subscriber _currentPoseSub;
	bool _isCanceled;
//----------------------------------------------------------------------------------------------------------chengyuen3/1
	ros::Subscriber _currentPoseOdomSub;
	ros::Subscriber _commandSub;
	int _idFlag;
//----------------------------------------------------------------------------------------------------------
  ros::Subscriber _goalSub;
//  ros::Subscriber _globalPlanSub;
	double _distanceBetweenGoal;
	//MoveBaseClient _action;
	void updateGoalsFromServer();
	bool setGoal(std::pair<double, std::pair<double, double> >& goal);
	void cancelGoal();
	// void cancelGoal(MoveBaseClient &ac);
	void clearGoals();
	void stopAction();
	// void handleCurrentPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
	// void currentPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	double getYaw(tf::Pose& t);
	double getDistanceBetweenGoal(tf::Pose& t);
	bool equalGoal(std::pair<double, std::pair<double, double> >& goal0, move_base_msgs::MoveBaseGoal& goal1);
	bool setGoalArrive();
//----------------------------------------------------------------------------------------------------------chengyuen3/1
	void resumeAction();
	void commandAction(const std_msgs::String::ConstPtr& cmd);
	void currentPoseReceivedOdom(const nav_msgs::OdometryConstPtr& msg);
	void handleCurrentPoseMessage(const nav_msgs::Odometry& msg);
//----------------------------------------------------------------------------------------------------------
  //callBack
  void getGoalCallback(const t3_description::goal& msg);
//  void getGlobalPlanCallback(const nav_msgs::Path& pathMsg);
};

//boost::shared_ptr<GoalsNode> goals_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  //amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goals");
	// Override default sigint handler
	signal(SIGINT, sigintHandler);
	GoalsNode *gn = new GoalsNode();
	gn->process();
	return 0;
}

std::string intToString (int v)  
{  
	std::ostringstream oss;
	oss << v;
	return oss.str();
} 

GoalsNode::GoalsNode() :
//----------------------------------------------------------------------------------------------------------chengyuen3/1
	ac("move_base", true),
	_idFlag(1),
//----------------------------------------------------------------------------------------------------------
	_arriveGoal(true),
	_distanceBetweenGoal(100.f),
  _currentIdxOfGoal(0),
  _isMapGoal(false)
{	
	//get param
	_privateNh.param("/goals/countOfGoals", _countOfGoals, 0);
	ROS_INFO("count: %d \n", _countOfGoals);
	_privateNh.param("frame_id", _frameId, std::string("/map"));
	//get goals
	updateGoalsFromServer();
	//set sub
	// _currentPoseSub = _nh.subscribe("amcl_pose", 2, &GoalsNode::currentPoseReceived, this);
//----------------------------------------------------------------------------------------------------------chengyuen3/1
	//_currentPoseOdomSub = _nh.subscribe("odometry_filtered_map", 2, &GoalsNode::currentPoseReceivedOdom, this);
	_commandSub = _nh.subscribe("command", 2, &GoalsNode::commandAction, this);
//----------------------------------------------------------------------------------------------------------
  _goalSub = _nh.subscribe("robotGoal",100, &GoalsNode::getGoalCallback, this);
//  _globalPlanSub = _nh.subscribe("TrajectoryPlannerROS/global_plan", 1000, &GoalsNode::getGlobalPlanCallback, this);

}

// GoalsNode::~GoalsNode()
// {

// }
	

void GoalsNode::updateGoalsFromServer()
{
	//get param goal*_x goal*_y goal*_z to _posesOfGoals
	for(int idx = 0; idx < _countOfGoals; idx++)
	{
		std::string goalNameX_ = "/goals/goal" + intToString(idx) + "_x";
		std::string goalNameY_ = "/goals/goal" + intToString(idx) + "_y";
		std::string goalNameZ_ = "/goals/goal" + intToString(idx) + "_z";
		double tempX_;
		double tempY_;
		double tempZ_;
		_privateNh.param(goalNameX_, tempX_, 0.0);
		_privateNh.param(goalNameY_, tempY_, 0.0);
		_privateNh.param(goalNameZ_, tempZ_, 0.0);
		
		if(!std::isnan(tempX_) && !std::isnan(tempY_) && !std::isnan(tempZ_))
		{
			_posesOfGoals.push_back(std::make_pair(tempZ_, std::make_pair(tempX_, tempY_)));
		}

		else
			ROS_WARN("ignoring NAN in goal_pose position");
		
	}
}

void GoalsNode::process()
{
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_WARN("Waiting for the move_base action server");
	}
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
    if(!_posesOfGoals.empty())
    {
      if(_isMapGoal)
      {
        setGoal(_posesOfGoals[_currentIdxOfGoal]);
        ac.sendGoalAndWait(_currentGoal);
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          //arriveGoal = true;
          _isMapGoal = false;
          ROS_INFO("You have arrived to the goal %d position", _currentIdxOfGoal);
          clearGoals();
        }else
        {
          ROS_WARN(" %d position error [ %s ]", _currentIdxOfGoal, ac.getState().toString().c_str());
        }
      }else
      {
        if(_countOfGoals <= _currentIdxOfGoal)
        {
          _currentIdxOfGoal = 0;
          _idFlag = 1;				// 1 means ++, 0 means --
        }

        setGoal(_posesOfGoals[_currentIdxOfGoal]);
        ac.sendGoalAndWait(_currentGoal);
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          //arriveGoal = true;
          ROS_INFO("You have arrived to the goal %d position", _currentIdxOfGoal);

          _currentIdxOfGoal++;
        }else
        {
          ROS_WARN(" %d position error [ %s ]", _currentIdxOfGoal, ac.getState().toString().c_str());
        }
      }

    }


		ros::spinOnce();
		loop_rate.sleep();		
	}

}

bool GoalsNode::setGoal(std::pair<double, std::pair<double, double> >& goal)
{
	// if(equalGoal(goal, _currentGoal))
	// {
	// 	// ROS_INFO("flase goal");
	// 	return false;
	// }
	std::pair<double, std::pair<double, double> > poseOfGoal_(goal);
	std::pair<double, double> locationOfGoal_(goal.second);
	double z = goal.first;
	double x = locationOfGoal_.first;
	double y = locationOfGoal_.second;

	_currentGoal.target_pose.header.stamp = ros::Time::now();
	_currentGoal.target_pose.header.frame_id = "map";
	_currentGoal.target_pose.pose.position.x = x;
	_currentGoal.target_pose.pose.position.y = y;
	_currentGoal.target_pose.pose.orientation.z = z;
	_currentGoal.target_pose.pose.orientation.w = 1.0;

	//ROS_INFO("Sending goal %d", idx);

	// ROS_INFO("true goal");
	return true;
}		

void GoalsNode::clearGoals()
{
	_posesOfGoals.clear();
  _currentIdxOfGoal = 0;
  _countOfGoals = 0;
  cancelGoal();
}

void GoalsNode::stopAction()
{
	_currentIdxOfGoal = 0;
	cancelGoal();
}

//----------------------------------------------------------------------------------------------------------chengyuen3/1
void GoalsNode::resumeAction()
{
	_isCanceled = false;
}

void GoalsNode::currentPoseReceivedOdom(const nav_msgs::OdometryConstPtr& msg)
{
	handleCurrentPoseMessage(*msg);
}

/********************callback********************/
//----------------------------------------------------------------------------------------------------------

/*
*note: callback of pose
*par:  msg: msg of pose
*/
// void GoalsNode::currentPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
// {
// 	handleCurrentPoseMessage(*msg);
// }

//----------------------------------------------------------------------------------------------------------chengyuen3/1
void GoalsNode::handleCurrentPoseMessage(const nav_msgs::Odometry& msg)
//----------------------------------------------------------------------------------------------------------
// void GoalsNode::handleCurrentPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	if(msg.header.frame_id == "")
  	{
    	// This should be removed at some point
    	ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  	}

  	tf::Pose currentPose_;
    tf::poseMsgToTF(msg.pose.pose, currentPose_);
	// ROS_INFO("current pose (%.6f): %.3f %.3f %.3f",
	//        ros::Time::now().toSec(),
	//        currentPose_.getOrigin().x(),
	//        currentPose_.getOrigin().y(),
	//        getYaw(currentPose_));
	_distanceBetweenGoal = getDistanceBetweenGoal(currentPose_);
	if(_distanceBetweenGoal < kMinDistance) {
//----------------------------------------------------------------------------------------------------------chengyuen3/1
		if (_idFlag == 1)
			_currentIdxOfGoal++;
		else if (_idFlag == 0)
			_currentIdxOfGoal--;
	}
//----------------------------------------------------------------------------------------------------------
}
///
/// \brief getGoalCallback
/// \param msg
///
void GoalsNode::getGoalCallback(const t3_description::goal& msg)
{
  _isMapGoal = true;
  clearGoals();//clear goal list
  _posesOfGoals.push_back(std::make_pair(msg.z, std::make_pair(msg.x, msg.y)));
  _countOfGoals++;
//  setGoal(clickGoal_);
}




//----------------------------------------------------------------------------------------------------------chengyuen3/1
void GoalsNode::cancelGoal()
//----------------------------------------------------------------------------------------------------------
// void GoalsNode::cancelGoal(MoveBaseClient &ac)
{
	_isCanceled = true;
	ac.cancelGoal();
}

double GoalsNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

double GoalsNode::getDistanceBetweenGoal(tf::Pose& t)
{
	double distance = 0.f;
	double goalX = 0.f;
	double goalY = 0.f;
	double poseX = 0.f;
	double poseY = 0.f;
	goalX = _currentGoal.target_pose.pose.position.x;
	goalY = _currentGoal.target_pose.pose.position.y;
	poseX = t.getOrigin().x();
	poseY = t.getOrigin().y();
	distance = sqrt(pow(goalX - poseX, 2.0) + pow(goalY - poseY,2.0));
	ROS_INFO("distance: %f", distance);
	return distance;
}

bool GoalsNode::equalGoal(std::pair<double, std::pair<double, double> >& goal0, move_base_msgs::MoveBaseGoal& goal1)
{
		// ROS_INFO("x: %.3f %.3f y:%.3f %.3f",
	 //       goal0.second.first,
	 //       goal1.target_pose.pose.position.x,
	 //       goal0.second.second,
	 //       goal1.target_pose.pose.position.y);
	if((goal0.second.first == goal1.target_pose.pose.position.x) 
		&& (goal0.second.second == goal1.target_pose.pose.position.y) 
		&& (goal0.first == goal1.target_pose.pose.orientation.z))
	{
		// ROS_INFO("equal true");
		return true;
	}else
	{
		// ROS_INFO("equal false");
		return false;
	}
}

bool GoalsNode::setGoalArrive()
{

}

//----------------------------------------------------------------------------------------------------------chengyuen3/1
void GoalsNode::commandAction(const std_msgs::String::ConstPtr& cmd)
{
	ROS_INFO("cmd msg receive: %s", cmd->data.c_str());
	if (cmd->data == "cancel") {
		ROS_INFO("cancel goal");
		cancelGoal();
	}
	else if (cmd->data == "pause") {
		ROS_INFO("pause action");
		stopAction();
	}
	else if (cmd->data == "clear") {
		ROS_INFO("clear all goals");
		clearGoals();
	}
	else if (cmd->data == "resume") {
		ROS_INFO("resume action");
		resumeAction();
	}

}
//----------------------------------------------------------------------------------------------------------
