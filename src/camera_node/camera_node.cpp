#include <iostream>

#include <highgui.h>
#include <cv.h>
#include <raspicam/raspicam_cv.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <common.hpp>

using namespace std;

void convert_mat_to_ros_image(cv::Mat& frame, sensor_msgs::msg::Image& msg)
{
	/* Convert OpenCV MAT image to ROS2 sensor_msg */
	set_now(msg.header.stamp);
	msg.header.frame_id = "camera_frame";
	msg.height = frame.cols;
	msg.width = frame.rows;
	msg.encoding = mat_type2encoding(frame.type());
	msg.is_bigendian = false;
	msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
	msg.data.assign(frame.datastart, frame.dataend);
}

int main(int argc, char * argv[])
{
	raspicam::RaspiCam_Cv camera;

	if(!camera.open()) {
		cout << "failed to open pi camera!" << endl;
		return 0;
	}

	cv::Mat frame;

	//Initialize ROS2 RCL
	rclcpp::init(argc, argv);

	auto node = rclcpp::node::Node::make_shared("camera");

	rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
	custom_qos_profile.depth = 7;
	auto image_publisher = node->create_publisher<sensor_msgs::msg::Image>("raw_image", custom_qos_profile);
	rclcpp::WallRate loop_rate(90);

	sensor_msgs::msg::Image msg;

	//Grab new image and publish it
	while(rclcpp::ok()) {
		camera.grab();
		camera.retrieve(frame);

		/* Conver MAT image and publish the ROS2 image */
		convert_mat_to_ros_image(frame, msg);		
		image_publisher->publish(msg);

		cv::imshow("pi camera", frame);			

		/* Leave if press any key */	
		if(cvWaitKey(1) != -1) {
			break;
		}
	}

	camera.release();

	return 0;
}
