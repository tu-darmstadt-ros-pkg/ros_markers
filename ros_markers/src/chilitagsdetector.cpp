#include "chilitagsdetector.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the markers transformation should be published?
// this allow to compensate for the 'slowness' of tag detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0

ChilitagsDetector::ChilitagsDetector(ros::NodeHandle& rosNode,
                                     const string& configFilename,
                                     bool omitOtherTags,
                                     double tagSize) :
            rosNode(rosNode),
            it(rosNode),
            firstUncalibratedImage(true),
            gotCameraInfo(false),
            chilitags3d(cv::Size(0,0)) // will call setDefaultTagSize with default chilitags parameter values

{

    sub = it.subscribeCamera("image", 1, &ChilitagsDetector::findMarkers, this);
    world_model_pub = rosNode.advertise<hector_worldmodel_msgs::PosePercept>("worldmodel/pose_percept", 5);

    if(!configFilename.empty()) {
    	chilitags3d.readTagConfiguration(configFilename, omitOtherTags);
    }

    if(tagSize!=USE_CHILITAGS_DEFAULT_PARAM)
        chilitags3d.setDefaultTagSize(tagSize); // use specified value

}


void ChilitagsDetector::setROSTransform(Matx44d trans, tf::Transform& transform)
{
    transform.setOrigin( tf::Vector3( trans(0,3) / 1000,
                                    trans(1,3) / 1000,
                                    trans(2,3) / 1000) );

    tf::Quaternion qrot;
    tf::Matrix3x3 mrot(
        trans(0,0), trans(0,1), trans(0,2),
        trans(1,0), trans(1,1), trans(1,2),
        trans(2,0), trans(2,1), trans(2,2));
    mrot.getRotation(qrot);
    transform.setRotation(qrot);
}

void ChilitagsDetector::publishPercept(const std::string& object_name, tf::Transform& transform){
    hector_worldmodel_msgs::PosePercept pp_msg;
    pp_msg.header.frame_id = cameramodel.tfFrame();
    pp_msg.header.stamp = ros::Time::now();
    pp_msg.info.class_id = "chilli_objects";
    pp_msg.info.class_support = 1.0;
    pp_msg.info.name = object_name;
    pp_msg.info.object_id = object_name;
    // would be great to create a function to estimate the certainty of the marker detection here
    // this could either be a one shot calculation based on the relative cossines of the marker and
    // camera as well as the pixel coordinates of the marker.
    // pixel coordinates = closer to centre the higher the score
    // relative cossines = more straight on the marker is detected the higher the score
    pp_msg.info.object_support = 1.0;
    pp_msg.pose.pose.position.x = transform.getOrigin().getX();
    pp_msg.pose.pose.position.y = transform.getOrigin().getY();
    pp_msg.pose.pose.position.z = transform.getOrigin().getZ();
    pp_msg.pose.pose.orientation.x = transform.getRotation().getX();
    pp_msg.pose.pose.orientation.y = transform.getRotation().getY();
    pp_msg.pose.pose.orientation.z = transform.getRotation().getZ();
    pp_msg.pose.pose.orientation.w = transform.getRotation().getW();
    world_model_pub.publish(pp_msg);
}

void ChilitagsDetector::findMarkers(const sensor_msgs::ImageConstPtr& msg, 
                                    const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
    if (!gotCameraInfo){
        cameramodel.fromCameraInfo(camerainfo);
        if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
            ROS_WARN("No valid intrinsic parameters can't detect markers");
            return;
        }
        chilitags3d.setCalibration(cameramodel.intrinsicMatrix(),
                                   cameramodel.distortionCoeffs());
        gotCameraInfo = true;
    }

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 

    /********************************************************************
    *                      Markers detection                           *
    ********************************************************************/

    auto foundObjects = chilitags3d.estimate(inputImage);
    ROS_INFO_STREAM(foundObjects.size() << " objects found.");

    /****************************************************************
    *                Publish TF transforms                          *
    *****************************************************************/

    objectsSeen.clear();

    for (auto& kv : foundObjects) {

        objectsSeen.insert(kv.first);
        setROSTransform(kv.second, 
                        transform);

        br.sendTransform(
                tf::StampedTransform(transform, 
                                        ros::Time::now() + ros::Duration(TRANSFORM_FUTURE_DATING), 
                                        cameramodel.tfFrame(),
                                        kv.first));

        publishPercept(kv.first, transform);
    }




}

