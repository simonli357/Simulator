
#include "gps_plugin.hpp"
#include <cmath>
#define DEBUG false

namespace gazebo
{
    namespace gps
    {   
        GPS::GPS():ModelPlugin() {
          
        }
     		
        void GPS::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
          nh = boost::make_shared<ros::NodeHandle>();
          nh->getParam("/rate", this->rate);
          nh->getParam("/max_noise", this->m_random_noise);
          double delay;
          nh->getParam("/gps_delay", delay);
          timer = nh->createTimer(ros::Duration(1/this->rate), std::bind(&GPS::OnUpdate, this));
          
          int init_size = std::round(delay * this->rate);
          for (int hsy=0; hsy<init_size; hsy++) {
            utils::localisation dummy;
            dummy.header.stamp = ros::Time::now();
            dummy.posA = 100.0;
            dummy.posB = 100.0;
            this->data_queue.push(dummy);
          }
          std::cout << "GPS Queue size: " << this->data_queue.size() << std::endl;

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

        	// Create topic name      	
        	std::string topic_name = namespace_ + "/localisation";
  	        
          // Initialize ros, if it has not already been initialized.
    			if (!ros::isInitialized())
    			{
      			  int argc = 0;
      			  char **argv = NULL;
      			  ros::init(argc, argv, "localisationNODEvirt", ros::init_options::NoSigintHandler);
    			}

          this->m_ros_node.reset(new ::ros::NodeHandle("/localisationNODEvirt"));

        	this->m_pubGPS = this->m_ros_node->advertise<utils::localisation>(topic_name, 2);
        	
          if(DEBUG)
          {
              std::cerr << "\n\n";
              ROS_INFO_STREAM("====================================================================");
              ROS_INFO_STREAM("[gps_plugin] attached to: " << this->m_model->GetName());
              ROS_INFO_STREAM("[gps_plugin] publish to: "  << topic_name);
              ROS_INFO_STREAM("[gps_plugin] Usefull data: linear x, linear y, angular z");
              ROS_INFO_STREAM("====================================================================\n\n");
          }
        }

        // Publish the updated values
        void GPS::OnUpdate()
        {
            this->m_gps_pose.header.stamp = ros::Time::now();
		        this->m_gps_pose.timestamp  = this->m_model->GetWorld()->SimTime().Float();
           	this->m_gps_pose.posA   = this->m_model->RelativePose().Pos().X() + (rand() / (float)RAND_MAX * this->m_random_noise) - this->m_random_noise/2;
           	this->m_gps_pose.posB   = abs(this->m_model->RelativePose().Pos().Y()) + (rand() / (float)RAND_MAX * this->m_random_noise) - this->m_random_noise/2;
           	this->m_gps_pose.rotA   = this->m_model->RelativePose().Rot().Yaw();
           	this->m_gps_pose.rotB   = this->m_model->RelativePose().Rot().Yaw();

            this->data_queue.push(this->m_gps_pose);
            utils::localisation* current = &this->data_queue.front();
            if (current->posA < 99.0) {
              this->m_pubGPS.publish(*current);
            } else {
              std::cout << "dummy found, queue size: " << this->data_queue.size() << std::endl;
            }
            this->data_queue.pop();
        };      
    }; //namespace trafficLight
    GZ_REGISTER_MODEL_PLUGIN(gps::GPS)
}; // namespace gazebo