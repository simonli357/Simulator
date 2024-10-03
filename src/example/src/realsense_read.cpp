#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_camera_publisher");
    ros::NodeHandle nh;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non-default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);

    // Start streaming with the given configuration
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);
    ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu", 1);

    try {
        while (ros::ok()) {
            // Wait for the next set of frames from the camera
            rs2::frameset frames = pipe.wait_for_frames();

            // Align the depth frame to color frame
            auto aligned_frames = align_to_color.process(frames);

            // Get the aligned frames
            rs2::frame aligned_color_frame = aligned_frames.get_color_frame();
            rs2::frame aligned_depth_frame = aligned_frames.get_depth_frame();
            rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
            rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);

            // Get the color and depth frames
            // rs2::frame color_frame = frames.get_color_frame();
            // rs2::frame depth_frame = frames.get_depth_frame();
            // rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
            // rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);

            // Convert RealSense frame to OpenCV Mat for color frame
            cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);

            // Convert RealSense frame to OpenCV Mat for depth frame
            cv::Mat depth_mat(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);

            // Convert gyro and accel frames to ROS Imu message
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_link";

            float *gyro_data = (float *)gyro_frame.get_data();
            imu_msg.angular_velocity.x = gyro_data[0];
            imu_msg.angular_velocity.y = gyro_data[1];
            imu_msg.angular_velocity.z = gyro_data[2];

            float *accel_data = (float *)accel_frame.get_data();
            imu_msg.linear_acceleration.x = accel_data[0];
            imu_msg.linear_acceleration.y = accel_data[1];
            imu_msg.linear_acceleration.z = accel_data[2];

            // Convert color Mat to ROS Image message
            sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_mat).toImageMsg();

            // Convert depth Mat to ROS Image message
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_mat).toImageMsg();

            // Publish color, depth, and imu data
            color_pub.publish(color_msg);
            depth_pub.publish(depth_msg);
            imu_pub.publish(imu_msg);

            ros::spinOnce();
        }
    } catch (const rs2::error &e) {
        ROS_ERROR_STREAM("RealSense error: " << e.what());
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("An error occurred: " << e.what());
    }

    return 0;
}
