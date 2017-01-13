/*******************************************************************************
//author: Arul Selvam Periyasamy
// mail id: arulselvam@uni-bonn.de
// This node subscribes to the images from the two pointgrey camera (vertical 
   by default), and the camera parameters and rotates the images and modifies 
// the camera parameters to fit the input needs of ros stereo_imag_proc
	
*******************************************************************************/
#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <camera_calibration_parsers/parse.h>

//topics to be subscribed
std::string g_left_image = "/stereo/left/image_raw";
std::string g_right_image = "/stereo/right/image_raw";
std::string g_left_cam_info = "/stereo/left/camera_info";
std::string g_right_cam_info = "/stereo/right/camera_info";


//topic to be published
std::string g_v_left_image = "/vertical_stereo/left/image_raw";
std::string g_v_right_image = "/vertical_stereo/right/image_raw";
std::string g_v_left_cam_info = "/vertical_stereo/left/camera_info";
std::string g_v_right_cam_info = "/vertical_stereo/right/camera_info";

// ros publishers
ros::Publisher pub_left_image;
ros::Publisher pub_right_image;
ros::Publisher pub_left_cam_info;
ros::Publisher pub_right_cam_info;
 

//  NOTE while publishing the modified data, and also frame id in the headers 
// so that the TF is clean 
void stereo_callback(const sensor_msgs::ImageConstPtr& left_msg,
					 const sensor_msgs::ImageConstPtr& right_msg,
					 const sensor_msgs::CameraInfoConstPtr& cam_info_left,
					 const sensor_msgs::CameraInfoConstPtr& cam_info_right
					) {
	cv_bridge::CvImagePtr left_img_ptr, right_img_ptr;
	try {
		left_img_ptr = cv_bridge::toCvCopy(left_msg,  sensor_msgs::image_encodings::MONO8 );
		right_img_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	std_msgs::Header left_image_header = left_msg->header;
	std_msgs::Header right_image_header = right_msg->header;
	
	left_image_header.frame_id = "vertical_" + left_image_header.frame_id;
	right_image_header.frame_id = "vertical_" + right_image_header.frame_id;
	
	// step 1: transpose left and right images
	cv::Mat left_image = left_img_ptr->image;
	cv::Mat right_image = right_img_ptr->image;
	
	cv::Mat left_image_T = left_image.t();
	cv::Mat right_image_T = right_image.t();

	// step 2: modify cam_info
	sensor_msgs::CameraInfo m_cam_info_left, m_cam_info_right;
	// copy the data
	m_cam_info_left = *cam_info_left;
	m_cam_info_right = *cam_info_right;
	// modify the data
	// The default structure of K with 9 elements as vector of 9 elements
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	// New structure needed
	//     [fy  0 cy]
	// K = [ 0 fx cx],
	//     [ 0  0  1]
	m_cam_info_left.K[0] = cam_info_left->K[4];
	m_cam_info_left.K[4] = cam_info_left->K[0];
	m_cam_info_left.K[2] = cam_info_left->K[5];
	m_cam_info_left.K[5] = cam_info_left->K[2];
	
	m_cam_info_right.K[0] = cam_info_right->K[4];
	m_cam_info_right.K[4] = cam_info_right->K[0];
	m_cam_info_right.K[2] = cam_info_right->K[5];
	m_cam_info_right.K[5] = cam_info_right->K[2];
	
	// The default structure of K with 12 elements as vector of 12 elements
	//     [fx'  0  cx' Tx]
	// P = [ 0  fy' cy' Ty]
	//     [ 0   0   1   0]
	// New structure needed
	//     [fy'  0  cy' Ty]
	// P = [ 0  fx' cx' Tx]
	//     [ 0   0   1   0]
	m_cam_info_left.P[0] = cam_info_left->P[5];
	m_cam_info_left.P[5] = cam_info_left->P[0];
	m_cam_info_left.P[2] = cam_info_left->P[6];
	m_cam_info_left.P[6] = cam_info_left->P[2];
	m_cam_info_left.P[3] = cam_info_left->P[7];
	m_cam_info_left.P[7] = cam_info_left->P[3];

	m_cam_info_right.P[0] = cam_info_right->P[5];
	m_cam_info_right.P[5] = cam_info_right->P[0];
	m_cam_info_right.P[2] = cam_info_right->P[6];
	m_cam_info_right.P[6] = cam_info_right->P[2];
	m_cam_info_right.P[3] = cam_info_right->P[7];
	m_cam_info_right.P[7] = cam_info_right->P[3];
	
	// swap the camera info image dimensions
	m_cam_info_left.width = cam_info_left->height;
	m_cam_info_left.height = cam_info_left->width;
	m_cam_info_right.width = cam_info_right->height;
	m_cam_info_right.height = cam_info_right->width;
	
	m_cam_info_left.header.frame_id = "vertical_" + m_cam_info_left.header.frame_id;
	m_cam_info_right.header.frame_id = "vertical_" + m_cam_info_right.header.frame_id;
	
	
	sensor_msgs::ImagePtr left_ptr = cv_bridge::CvImage(left_image_header, "mono8", left_image_T).toImageMsg();
	sensor_msgs::ImagePtr right_ptr = cv_bridge::CvImage(right_image_header, "mono8", right_image_T).toImageMsg();
	pub_left_image.publish(left_ptr);
	pub_right_image.publish(right_ptr);
	pub_left_cam_info.publish(m_cam_info_left);
	pub_right_cam_info.publish(m_cam_info_right);
}


int main(int argc, char **argv){
ros::init(argc, argv, "vertical_stereo");
	ros::NodeHandle nh;
	
    message_filters::Subscriber<sensor_msgs::Image> leftImage(nh, g_left_image, 1);
	message_filters::Subscriber<sensor_msgs::Image> rightImage(nh, g_right_image, 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> leftCamInfo(nh, g_left_cam_info , 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> rightCamInfo(nh, g_right_cam_info , 1);
	typedef message_filters::sync_policies::ApproximateTime
									<sensor_msgs::Image,
									sensor_msgs::Image,
									sensor_msgs::CameraInfo,
									sensor_msgs::CameraInfo> stereoSyncPolicy;
	message_filters::Synchronizer<stereoSyncPolicy> sync(stereoSyncPolicy(100), leftImage, rightImage, leftCamInfo, rightCamInfo);
	pub_left_image  = nh.advertise<sensor_msgs::Image>( g_v_left_image, 1 );
	pub_right_image  = nh.advertise<sensor_msgs::Image>( g_v_right_image, 1 );
	pub_left_cam_info  = nh.advertise<sensor_msgs::CameraInfo>( g_v_left_cam_info, 1 );
	pub_right_cam_info  = nh.advertise<sensor_msgs::CameraInfo>( g_v_right_cam_info, 1 );

	sync.registerCallback(boost::bind(&stereo_callback, _1, _2, _3, _4));
	
	std::cout<<" vertical stereo running"<<std::endl;
	ros::spin();
	return 0;
}
