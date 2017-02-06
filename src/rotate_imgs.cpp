/*******************************************************************************
//author: Arul Selvam Periyasamy
// mail id: arulselvam@uni-bonn.de
// This node subscribes to the images from the two pointgrey camera (vertical 
   by default), and the camera parameters and rotates the images and modifies 
// the camera parameters to fit the input needs of ros stereo_imag_proc
	
*******************************************************************************/
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>
#include <typeinfo>


typedef message_filters::sync_policies::ApproximateTime
								<sensor_msgs::Image,
								sensor_msgs::Image> stereoSyncPolicy;
//topics to be subscribedright_img_ptr
std::string g_left_image = "/stereo/left/image_raw";
std::string g_right_image = "/stereo/right/image_raw";
std::string g_depth_image = "/pico_flexx/image_mono8";

//topic to be published
std::string g_v_left_image = "/vertical_stereo/left/image_raw";
std::string g_v_right_image = "/vertical_stereo/right/image_raw";

// ros publishers
ros::Publisher pub_left_image;
ros::Publisher pub_right_image;

bool g_rotateCCWLeft, g_rotateCCWRight,
	 g_rotateCWLeft, g_rotateCWRight,
	 g_rotate180Left, g_rotate180Right;

sensor_msgs::ImagePtr rotateImg( const sensor_msgs::ImageConstPtr & msg, 
								 const bool rotateCCW,
								 const bool rotateCW, 
								 const bool rotate180 ){
	
	std_msgs::Header new_image_header = msg->header;
	new_image_header.frame_id = "vertical_" + new_image_header.frame_id;

	
	cv_bridge::CvImagePtr img_ptr;
	try {
		img_ptr = cv_bridge::toCvCopy( msg, msg->encoding );
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return sensor_msgs::ImagePtr(nullptr);
	}

	cv::Mat tmp = img_ptr->image;
	cv::Mat new_image(tmp.cols, tmp.rows, tmp.type());
	
	if ( rotateCW ) {
		// rotates CW 90 degree
		cv::transpose( tmp, new_image ); // shouldn't be done inplace
		new_image.copyTo(tmp);
		cv::flip( tmp, new_image, 1);
	}
	else if ( rotateCCW ) {
		// rotates CCW 90 degree
		cv::transpose( tmp, new_image ); // shouldn't be done inplace
		new_image.copyTo(tmp);
		cv::flip(tmp, new_image, 0);
	}
	else if ( rotate180 ) {
		cv::flip( tmp, new_image, -1);
	}
	else
		new_image = tmp;
	
	//   tmp = img_ptr->image;workOnBag
	//   ROS_INFO_STREAM("size: orig: " << tmp.rows<<"x"<< tmp.cols<<" ni: "<< new_image.rows << "x" << new_image.cols << " rotateCCW: " << rotateCCW << " rotateCW: " << rotateCW );
	
	//   cv::namedWindow("original",CV_WINDOW_AUTOSIZE);
	//   cv::namedWindow("rotated",CV_WINDOW_AUTOSIZE);
	//   cv::imshow("original",tmp);
	//   cv::imshow("rotated",new_image);
	//   cv::waitKey(0);

	return cv_bridge::CvImage( new_image_header, "mono8", new_image).toImageMsg();
}


void workOnBag ( const std::string & bagFileName, const std::string & otherBagFileName,
				 const bool rotateCCWLeft, const bool rotateCCWRight, 
				 const bool rotateCWLeft, const bool rotateCWRight, 
				 const bool rotate180Left, const bool rotate180Right ){
	
	// open bag file
	rosbag::Bag bag, saveBag;
	bag.open( bagFileName, rosbag::bagmode::Read);
	saveBag.open( otherBagFileName, rosbag::bagmode::Write);
	ROS_INFO_STREAM("Opened " << otherBagFileName << " und " << bagFileName  );

	// Get topics
	std::vector<std::string> topics;
	topics.push_back( g_left_image );
	topics.push_back( g_right_image );
	topics.push_back( g_depth_image );

	rosbag::View view(bag, rosbag::TopicQuery(topics));
	for ( rosbag::View::iterator it = view.begin(); it != view.end(); ++it )
	{
	if ( ! ros::ok() )
		break;

	rosbag::MessageInstance & m = *it;

	// get those with corresponding mTimeStamp
	sensor_msgs::ImageConstPtr msg = m.instantiate<sensor_msgs::Image>();
	if ( msg == nullptr )
	{
		ROS_ERROR_STREAM("couldn't instantiate as image: " << m.getTopic());
		saveBag.write(m.getTopic(),m.getTime(), m); // other messages are forwarded
		continue;
	}
	sensor_msgs::ImagePtr newMessage;
	if ( m.getTopic() == g_left_image )
	{
		newMessage = rotateImg(msg, rotateCCWLeft, rotateCWLeft, rotate180Left);
	}
	if ( m.getTopic() == g_right_image )
	{
		newMessage = rotateImg(msg, rotateCCWRight, rotateCWRight, rotate180Right );
	}
	if ( m.getTopic() == g_depth_image )
	{
		newMessage = rotateImg(msg, false, false, false);
	}
	saveBag.write<sensor_msgs::Image>(m.getTopic(),m.getTime(),newMessage);
	}
}

void stereo_callback(const sensor_msgs::ImageConstPtr& left_msg,
					 const sensor_msgs::ImageConstPtr& right_msg
					) {
	
	sensor_msgs::ImagePtr v_left_msg, v_right_msg;
	v_left_msg = rotateImg(left_msg, g_rotateCCWLeft, g_rotateCWLeft, g_rotate180Left);
	v_right_msg =  rotateImg(right_msg, g_rotateCCWRight, g_rotateCWRight, g_rotate180Right);
	pub_left_image.publish(v_left_msg);
	pub_right_image.publish(v_right_msg);
	
}


int main(int argc, char **argv){
	ros::init(argc, argv, "rotate_imgs");
	ros::NodeHandle nh;

	bool rotateCCWLeft = false; // rotate by 90 degree CCW e.g. by vert flip + transpose
	bool rotateCCWRight = true;
	
	bool rotateCWLeft = true; // rotate by 90 degree CW e.g. transpose matrix
	bool rotateCWRight = false;
	
	bool rotate180Left = false; // rotate by 90 degree CW e.g. transpose matrix
	bool rotate180Right = false;
	
	bool workOnBagOnly = false;
	
	std::string bagFileName="/home/ljquenzel/mario_calib.bag";
	std::string saveBagFileName="/home/ljquenzel/rotated_calib_1.bag";
	
	nh.param<bool>( "rotateCCWLeft", rotateCCWLeft, rotateCCWLeft);
	nh.param<bool>( "rotateCCWRight", rotateCCWRight, rotateCCWRight);
	
	nh.param<bool>( "rotate180Left", rotate180Left, rotate180Left);
	nh.param<bool>( "rotate180Right", rotate180Right, rotate180Right);
	
	nh.param<bool>( "rotateCWLeft", rotateCWLeft, rotateCWLeft);
	nh.param<bool>( "rotateCWRight", rotateCWRight, rotateCWRight);
	
	nh.param<bool>( "workOnBagOnly", workOnBagOnly, workOnBagOnly);
	//nh.param<std::string>( "bagFileName", bagFileName, bagFileName);
	//nh.param<std::string>( "saveBagFileName", saveBagFileName, saveBagFileName);
	
	ROS_INFO_STREAM( "rotate:"<< rotateCCWLeft << rotateCCWRight << rotateCWLeft
	<< rotateCWRight << rotate180Left << rotate180Right );
	
	if ( workOnBagOnly ){
		ROS_INFO_STREAM("Now starting to process.");
		
		workOnBag( bagFileName, saveBagFileName, rotateCCWLeft, rotateCCWRight,
				   rotateCWLeft, rotateCWRight, rotate180Left, rotate180Right );
		
		ROS_INFO_STREAM("saved everything to bag.");
	} else {
		g_rotateCCWLeft = rotateCCWLeft;
		g_rotateCCWRight = rotateCCWRight;
		g_rotateCWLeft = rotateCWLeft;
		g_rotateCWRight = rotateCWRight;
		g_rotate180Left = rotate180Left;
		g_rotate180Right = rotate180Right;
		message_filters::Subscriber<sensor_msgs::Image> leftImage(nh, g_left_image, 1);
		message_filters::Subscriber<sensor_msgs::Image> rightImage(nh, g_right_image, 1);
		
		auto sync = std::make_shared<message_filters::Synchronizer<stereoSyncPolicy>>(
			stereoSyncPolicy(100),leftImage, rightImage
		);
		pub_left_image  = nh.advertise<sensor_msgs::Image>( g_v_left_image, 1 );
		pub_right_image  = nh.advertise<sensor_msgs::Image>( g_v_right_image, 1 );
		
		sync->registerCallback(boost::bind(&stereo_callback, _1, _2));
		ros::spin();
	}
	return 0;
}
