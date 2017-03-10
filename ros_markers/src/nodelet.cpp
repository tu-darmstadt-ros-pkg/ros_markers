#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "chilitagsdetector.hpp"

#include <actionlib/server/simple_action_server.h>
#include <ros_markers_msgs/RosMarkersAction.h>


typedef actionlib::SimpleActionServer<ros_markers_msgs::RosMarkersAction> RosMarkersActionServer;


using namespace std;

/** @file

    @brief ROS nodelet for Chilitags marker detection.

*/

class ChilitagsNodelet: public nodelet::Nodelet
{
public:
  ChilitagsNodelet()
  {}

  ~ChilitagsNodelet()
  {
    NODELET_INFO("Stopping chilitags detection");
  }

  void rosMarkersDetectionGoalCallback();
  void rosMarkersDetectionPreemptCallback();

private:
  virtual void onInit();
  boost::shared_ptr<ChilitagsDetector> chilitags_;
  boost::shared_ptr<RosMarkersActionServer> ros_markers_as_ptr_;
  RosMarkersActionServer::GoalConstPtr ros_markers_goal_ptr_;


  ros::NodeHandle rosNode;
  string configFilename;
  string tfParentFrame;
  double defaultTagSize;
  int persistance;
  double filterGain;
  bool omitOtherTags;
};

void ChilitagsNodelet::onInit()
{
    //ROS initialization
    rosNode = getNodeHandle();
    ros::NodeHandle _private_node(getPrivateNodeHandle());

    // load parameters
    _private_node.param<string>("markers_configuration", configFilename, "");
    _private_node.param<string>("tf_parent_frame", tfParentFrame, "");
    _private_node.param<double>("default_marker_size", defaultTagSize, USE_CHILITAGS_DEFAULT_PARAM);
    _private_node.param<double>("gain", filterGain, -1.0);
    _private_node.param<int>("persistance", persistance, -1.0);
    _private_node.param<bool>("omit_other_tags", omitOtherTags, false);

    if (configFilename.empty() && omitOtherTags == true) {
        NODELET_ERROR_STREAM("If a marker configuration file is not passed as a parameter,\n" <<
                             "omitOtherTags must not be set to true or no tags will be detected.");
    }

    ros_markers_as_ptr_.reset(new RosMarkersActionServer(_private_node, "ros_markers_detection", 0, false));

    ros_markers_as_ptr_->registerGoalCallback(boost::bind(&ChilitagsNodelet::rosMarkersDetectionGoalCallback, this));
    ros_markers_as_ptr_->registerPreemptCallback(boost::bind(&ChilitagsNodelet::rosMarkersDetectionPreemptCallback, this));

    ros_markers_as_ptr_->start();
}

void ChilitagsNodelet::rosMarkersDetectionGoalCallback(){
    ros_markers_goal_ptr_ = ros_markers_as_ptr_->acceptNewGoal();
    // initialize the detector by subscribing to the camera video stream
    chilitags_.reset(new ChilitagsDetector(rosNode, configFilename, omitOtherTags, defaultTagSize, tfParentFrame, persistance, filterGain));
    ROS_INFO("ros_markers nodelet is ready. Marker locations will be published on TF when detected.");
}

void ChilitagsNodelet::rosMarkersDetectionPreemptCallback(){
    ros_markers_as_ptr_->setPreempted();
    chilitags_.reset();
    NODELET_INFO("Stopping to look for chilitags.");
}

// Register this plugin with pluginlib.  Names must match nodelet_chilitags.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(ros_markers, detector,
                        ChilitagsNodelet, nodelet::Nodelet);
