#include "bno055_plugin.hpp"
#include <random> 
#include <cmath>
#define DEBUG false

namespace gazebo
{
    namespace bno055
    {   
        BNO055::BNO055():ModelPlugin() {
            prev_linear_velocity = ignition::math::Vector3d(0, 0, 0);  // Assuming you're using Gazebo 9 or newer
            prev_time = ros::Time::now();
        }

        void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            nh = boost::make_shared<ros::NodeHandle>();	
            timer = nh->createTimer(ros::Duration(0.02), std::bind(&BNO055::OnUpdate, this));
	    	
      			// Save a pointer to the model for later use
      			this->m_model = model_ptr;

            std::string namespace_ = "";
            if (sdf_ptr->HasElement("rosTopicNamespace"))
            {
                namespace_ = sdf_ptr->Get<std::string>("rosTopicNamespace");
            }

            if (!namespace_.empty() && namespace_.front() != '/')
            {
                namespace_ = "/" + namespace_;
            }
            ROS_INFO_STREAM("imu Namespace: " << namespace_);
          	// Create topic name        	
          	std::string topic_name = namespace_ + "/IMU";
	        

            // Initialize ros, if it has not already bee initialized.
      			if (!ros::isInitialized())
      			{
        			  int argc = 0;
        			  char **argv = NULL;
        			  ros::init(argc, argv, "gazebo_client_bno", ros::init_options::NoSigintHandler);
      			}

            this->m_ros_node.reset(new ::ros::NodeHandle("/bnoNODEvirt"));
          	this->m_pubBNO = this->m_ros_node->advertise<utils::IMU>(topic_name, 2);
            this->m_pubIMU = this->m_ros_node->advertise<sensor_msgs::Imu>(namespace_ + "/imu", 2);
            // this->m_pubEncoder = this->m_ros_node->advertise<utils::encoder>(namespace_ + "/encoder", 2);

            if(DEBUG)
            {
                std::cerr << "\n\n";
                ROS_INFO_STREAM("====================================================================");
                ROS_INFO_STREAM("[bno055_plugin] attached to: " << this->m_model->GetName());
                ROS_INFO_STREAM("[bno055_plugin] publish to: "  << topic_name);
                ROS_INFO_STREAM("[bno055_plugin] Usefull data: linear z, angular x, angular y, angular z");
                ROS_INFO_STREAM("====================================================================");
            }
        }

        // Publish the updated values
        void BNO055::OnUpdate()
        {        
            this->m_bno055_pose.header.stamp = ros::Time::now();
            this->m_bno055_pose.header.frame_id = "bno055";
           	this->m_bno055_pose.roll = this->m_model->RelativePose().Rot().Roll();
		        this->m_bno055_pose.pitch = this->m_model->RelativePose().Rot().Pitch();
           	this->m_bno055_pose.yaw = this->m_model->RelativePose().Rot().Yaw();
            this->m_pubBNO.publish(this->m_bno055_pose);

            std::default_random_engine generator;
            std::normal_distribution<double> distribution(0.0, 1.0);  // mean=0, standard deviation=1

            this->m_imu_msg.header.stamp = ros::Time::now();
            this->m_imu_msg.header.frame_id = "chassis";

            // orientation
            this->m_imu_msg.orientation.x = this->m_model->RelativePose().Rot().X() ;//+ 0.05 * distribution(generator);  // Added Gaussian noise
            this->m_imu_msg.orientation.y = this->m_model->RelativePose().Rot().Y() ;//+ 0.05 * distribution(generator);
            this->m_imu_msg.orientation.z = this->m_model->RelativePose().Rot().Z() ;//+ 0.05 * distribution(generator);
            this->m_imu_msg.orientation.w = this->m_model->RelativePose().Rot().W() ;//+ 0.05 * distribution(generator);
            //variance is 0.01 squared. use power function
            double orientation_variance = pow(0.01, 2);
            this->m_imu_msg.orientation_covariance = {orientation_variance, 0, 0, 
                                                      0, orientation_variance, 0, 
                                                      0, 0, orientation_variance};

            // angular velocity
            ignition::math::Vector3d angular_velocity = this->m_model->WorldAngularVel();
            this->m_imu_msg.angular_velocity.x = angular_velocity.X() ;//+ 0.1 * distribution(generator);
            this->m_imu_msg.angular_velocity.y = angular_velocity.Y() ;//+ 0.1 * distribution(generator);
            this->m_imu_msg.angular_velocity.z = angular_velocity.Z() ;//+ 0.1 * distribution(generator);
            double angular_velocity_variance = pow(0.03, 2); 
            this->m_imu_msg.angular_velocity_covariance = {angular_velocity_variance, 0, 0, 
                                                           0, angular_velocity_variance, 0, 
                                                           0, 0, angular_velocity_variance};

            // linear acceleration
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - prev_time).toSec();
            ignition::math::Vector3d current_linear_velocity = this->m_model->WorldLinearVel();
            ignition::math::Vector3d linear_acceleration = (current_linear_velocity - prev_linear_velocity) / dt;  // Using ignition::math::Vector3d
            prev_linear_velocity = current_linear_velocity;
            prev_time = current_time;
            this->m_imu_msg.linear_acceleration.x = linear_acceleration.X();
            this->m_imu_msg.linear_acceleration.y = linear_acceleration.Y();
            this->m_imu_msg.linear_acceleration.z = linear_acceleration.Z();
            double linear_acceleration_variance = pow(0.04, 2);
            this->m_imu_msg.linear_acceleration_covariance = {linear_acceleration_variance, 0, 0, 
                                                              0, linear_acceleration_variance, 0, 
                                                              0, 0, linear_acceleration_variance};
            this->m_pubIMU.publish(this->m_imu_msg);

            // encoder
            // this->m_encoder_msg.header.stamp = ros::Time::now();
            // this->m_encoder_msg.header.frame_id = "encoder";
            // double x_speed = current_linear_velocity.X();
            // double y_speed = current_linear_velocity.Y();
            // double speedYaw = atan2(y_speed, x_speed);
            // double speed = sqrt(x_speed * x_speed + y_speed * y_speed);
            // double yaw = this->m_model->RelativePose().Rot().Yaw();
            // double angle_diff = fmod((speedYaw - yaw + M_PI), (2 * M_PI)) - M_PI;
            // if (fabs(angle_diff) > 3 * M_PI / 4)
            // {
            //     speed *= -1;
            // }
            // this->m_encoder_msg.speed = speed;
            // this->m_pubEncoder.publish(this->m_encoder_msg);
        };      
    }; 
    GZ_REGISTER_MODEL_PLUGIN(bno055::BNO055)
};
