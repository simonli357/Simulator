#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


#include "ros/ros.h"
#include "utils/localisation.h"

#include <fstream>
#include <string>
#include <sstream>

#include <queue>
#include <vector>

namespace gazebo
{
    namespace gps
    {   
        class GPS: public ModelPlugin
    	{
        private: 
            double rate;
            double m_random_noise;
            physics::ModelPtr                   m_model;
            ros::NodeHandlePtr		  nh;
            ros::Timer				  timer;
            std::queue<utils::localisation> data_queue;

	    /** ROS INTEGRATION **/
            // A node use for ROS transport
            std::unique_ptr<ros::NodeHandle>    m_ros_node;

            // A ROS publisher
            ros::Publisher                      m_pubGPS;

            // The gps message
            utils::localisation            m_gps_pose;
            utils::localisation            current_pose;
            
        // Default constructor
        public: GPS();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();
        public: void ReadConfig(const std::string &filename);        
        };
    };    
};