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

#include <math.h>
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
	std::string _frameId;
	int _countOfGoals;
	int _currentIdxOfGoal;
	bool _arriveGoal;
	//all poses of goals
	std::vector<std::pair<double, std::pair<double, double> > > _posesOfGoals;
	//std::vector<double> _posesOfGoals;
	//tf::Pose _posesOfGoals;
	ros::NodeHandle _nh;
	ros::NodeHandle _privateNh;
	move_base_msgs::MoveBaseGoal _currentGoal;
	move_base_msgs::MoveBaseGoal _lastGoal;
	// ros::Subscriber _currentPoseSub;
//----------------------------------------------------------------------------------------------------------chengyuen3/1
	ros::Subscriber _currentPoseOdomSub;
	ros::Subscriber _commandSub;
	int _idFlag;
//----------------------------------------------------------------------------------------------------------
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
	_currentIdxOfGoal(0)
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
			// std::stringstream ss;
			// ss << goalNameX_ << ":" << tempX_ << " " << goalNameY_ << ":" << tempY_ << " " << goalNameZ_ << ":" << tempZ_;
			// ROS_INFO("%s", ss.str().c_str());
		}

		else
			ROS_WARN("ignoring NAN in goal_pose position");
		
	}
}

void GoalsNode::process()
{
	// ac("move_base", true);
	// MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_WARN("Waiting for the move_base action server");
	}
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		// if(arriveGoal)
		// {
//----------------------------------------------------------------------------------------------------------chengyuen3/1
		// switch (roundTrip) {
		// 	case 1:
		// 		if(_countOfGoals <= _currentIdxOfGoal)
		// 		{
		// 			_currentIdxOfGoal = 0;
		// 			_idFlag = 1;				// 1 means ++, 0 means --
		// 		}
		// 		break;
		// 	case 0:
		// 		if(_countOfGoals <= _currentIdxOfGoal)
		// 		{
		// 			_currentIdxOfGoal = _countOfGoals - 2;
		// 			_idFlag = 0;
		// 		}
		// 		else if((_currentIdxOfGoal < 0))
		// 		{
		// 			_currentIdxOfGoal = 1;
		// 			_idFlag = 1;
		// 		}
		// 		break;
		// }
//----------------------------------------------------------------------------------------------------------
			
//<<<<<<< master
			if(_countOfGoals <= _currentIdxOfGoal)
				{
					_currentIdxOfGoal = 0;
					_idFlag = 1;				// 1 means ++, 0 means --
				}


//			if(setGoal(_posesOfGoals[_currentIdxOfGoal]))
//			{
				 // ac.sendGoal(_currentGoal);
//				ac.sendGoalAndWait(_currentGoal);
//			}
			
//=======
			// if(setGoal(_posesOfGoals[_currentIdxOfGoal]))
			// {
			// 	 ac.sendGoal(_currentGoal);
			// }
			setGoal(_posesOfGoals[_currentIdxOfGoal]);
			ac.sendGoalAndWait(_currentGoal);
//>>>>>>> master
			// std::pair<double, std::pair<double, double> > poseOfGoal_(_posesOfGoals[idx]);
			// std::pair<double, double> locationOfGoal_(poseOfGoal_.second);
			// double z = poseOfGoal_.first;
			// double x = locationOfGoal_.first;
			// double y = locationOfGoal_.second;

			// _currentGoal.target_pose.header.stamp = ros::Time::now();
			// _currentGoal.target_pose.header.frame_id = "map";
			// _currentGoal.target_pose.pose.position.x = x;
			// _currentGoal.target_pose.pose.position.y = y;
			// _currentGoal.target_pose.pose.orientation.z = z;
			// _currentGoal.target_pose.pose.orientation.w = 1.0;

			// ROS_INFO("Sending goal %d", idx);

			// ac.sendGoal(_currentGoal);
		// }


		//ac.waitForResult();
		
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			
			//arriveGoal = true;
			ROS_INFO("You have arrived to the goal %d position", _currentIdxOfGoal);
			_currentIdxOfGoal++;	
		}else
		{
			ROS_WARN(" %d position error [ %s ]", _currentIdxOfGoal, ac.getState().toString().c_str());
		}

//-------------------------------------------------------------------------------------------------------------chengyuen4/1
		/*
		else if (ac.getState() == actionlib::SimpleClientGoalState::"please fill in here") {
			ros::Duration(durationSlp).sleep(); 
			ac.sendGoal(_currentGoal);
		}
	
		*/
//-------------------------------------------------------------------------------------------------------------

		// else
		// {
		// 	arriveGoal = false;
		// 	//ROS_INFO("The base failed for some reason");
		// }
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
	stopAction();
	_posesOfGoals.clear();
}

void GoalsNode::stopAction()
{

}

//----------------------------------------------------------------------------------------------------------chengyuen3/1
void GoalsNode::resumeAction()
{

}

void GoalsNode::currentPoseReceivedOdom(const nav_msgs::OdometryConstPtr& msg)
{
	handleCurrentPoseMessage(*msg);
}
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


//----------------------------------------------------------------------------------------------------------chengyuen3/1
void GoalsNode::cancelGoal()
//----------------------------------------------------------------------------------------------------------
// void GoalsNode::cancelGoal(MoveBaseClient &ac)
{
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