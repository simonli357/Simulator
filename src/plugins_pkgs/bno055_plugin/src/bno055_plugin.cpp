#include "bno055_plugin.hpp"
#include <random>
#include <cmath>
#define DEBUG false

namespace gazebo
{
  namespace bno055
  {
    //— Noise parameters tuned to BNO055 datasheet —
    constexpr double EXAGGERATION_FACTOR = 5.0; // 5x noise
    constexpr double ORIENTATION_NOISE_STD = 0.0624524 * M_PI / 180.0 * EXAGGERATION_FACTOR;   // rad  (0.0625° resolution)
    constexpr double ANGVEL_NOISE_STD      = 0.1002676 * M_PI / 180.0 * EXAGGERATION_FACTOR;   // rad/s (≈0.1°/s RMS)
    constexpr double LINACC_NOISE_STD      = 0.0104 * EXAGGERATION_FACTOR;    // m/s²  (150 μg/√Hz @~50Hz BW)
    constexpr double ENCODER_SPEED_NOISE_STD = 0.01047 * EXAGGERATION_FACTOR; // m/s (≈1 cm/s RMS)

    constexpr double ORIENTATION_NOISE_VAR = ORIENTATION_NOISE_STD * ORIENTATION_NOISE_STD;
    constexpr double ANGVEL_NOISE_VAR      = ANGVEL_NOISE_STD      * ANGVEL_NOISE_STD;
    constexpr double LINACC_NOISE_VAR      = LINACC_NOISE_STD      * LINACC_NOISE_STD;
    constexpr double ENCODER_SPEED_NOISE_VAR = ENCODER_SPEED_NOISE_STD * ENCODER_SPEED_NOISE_STD;

    BNO055::BNO055(): ModelPlugin()
    {
      prev_linear_velocity = ignition::math::Vector3d(0,0,0);
      prev_time            = ros::Time::now();
    }

    void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
    {
      this->m_model = model_ptr;

      // ROS init & publishers
      if (!ros::isInitialized()) {
        int argc = 0; char **argv = nullptr;
        ros::init(argc, argv, "gazebo_client_bno",
                  ros::init_options::NoSigintHandler);
      }
      this->m_ros_node.reset(new ros::NodeHandle("/bnoNODEvirt"));

      std::string ns = "";
      if (sdf_ptr->HasElement("rosTopicNamespace"))
        ns = sdf_ptr->Get<std::string>("rosTopicNamespace");
      if (!ns.empty() && ns.front() != '/') ns = "/" + ns;

      ROS_INFO_STREAM("imu Namespace: " << ns);
      this->m_pubBNO = this->m_ros_node->advertise<utils::IMU>(ns + "/IMU", 2);
      this->m_pubIMU = this->m_ros_node->advertise<sensor_msgs::Imu>(ns + "/imu", 2);
      this->m_pubEncoder = this->m_ros_node->advertise<utils::encoder>(ns + "/encoder", 2);

      // Schedule updates at 50 Hz (20 ms)
      double updateRate = 10.0;
      nh    = boost::make_shared<ros::NodeHandle>();
      timer = nh->createTimer(ros::Duration(1.0/updateRate),
                              std::bind(&BNO055::OnUpdate, this));
    }

    void BNO055::OnUpdate()
    {
      // 1) Publish raw BNO orientation (roll,pitch,yaw)
      m_bno055_pose.header.stamp    = ros::Time::now();
      m_bno055_pose.header.frame_id = "bno055";
      auto rot = m_model->RelativePose().Rot();
      m_bno055_pose.roll  = rot.Roll();
      m_bno055_pose.pitch = rot.Pitch();
      m_bno055_pose.yaw   = rot.Yaw();
      m_pubBNO.publish(m_bno055_pose);

      // 2) Fill standard Imu message with fused data + noise
      //    Units: orientation quaternion [unitless], angular velocity [rad/s], linear acc [m/s2]
      static std::default_random_engine         gen{ std::random_device{}() };
      std::normal_distribution<double> distO(0.0, ORIENTATION_NOISE_STD),
                                   distG(0.0, ANGVEL_NOISE_STD),
                                   distA(0.0, LINACC_NOISE_STD);

      // Header
      m_imu_msg.header.stamp    = ros::Time::now();
      m_imu_msg.header.frame_id = "chassis";

      auto v  = m_model->WorldLinearVel();
      double x_speed = v.X();
      double y_speed = v.Y();
      double speedYaw = std::atan2(y_speed, x_speed);
      double speed = std::sqrt(x_speed*x_speed + y_speed*y_speed);

      // Orientation (quaternion) + noise
      double drift_yaw;
      if (std::abs(speed) < 0.03) {
          drift_yaw = rot.Yaw();
      } else {
          drift += (5 * M_PI / 180) / 60 / 10;
          drift_yaw = rot.Yaw() + drift;
      }
      ignition::math::Quaterniond drift_q(0.0, 0.0, drift_yaw);

      // auto q = rot;  // using same Rot() for quat (X,Y,Z,W)
      m_imu_msg.orientation.x = drift_q.X() + distO(gen);
      m_imu_msg.orientation.y = drift_q.Y() + distO(gen);
      m_imu_msg.orientation.z = drift_q.Z() + distO(gen);
      m_imu_msg.orientation.w = drift_q.W() + distO(gen);
      m_imu_msg.orientation_covariance = {
        ORIENTATION_NOISE_VAR, 0, 0,
        0, ORIENTATION_NOISE_VAR, 0,
        0, 0, ORIENTATION_NOISE_VAR
      };

      // Angular velocity + noise
      auto w = m_model->WorldAngularVel();
      m_imu_msg.angular_velocity.x = w.X() + distG(gen);
      m_imu_msg.angular_velocity.y = w.Y() + distG(gen);
      m_imu_msg.angular_velocity.z = w.Z() + distG(gen);
      m_imu_msg.angular_velocity_covariance = {
        ANGVEL_NOISE_VAR, 0, 0,
        0, ANGVEL_NOISE_VAR, 0,
        0, 0, ANGVEL_NOISE_VAR
      };

      // Linear acceleration: derivative of velocity + noise
      ros::Time now = ros::Time::now();
      double dt = (now - prev_time).toSec();
      auto a  = (v - prev_linear_velocity) / (dt > 0 ? dt : 1e-6);
      prev_linear_velocity = v;
      prev_time            = now;
      m_imu_msg.linear_acceleration.x = a.X() + distA(gen);
      m_imu_msg.linear_acceleration.y = a.Y() + distA(gen);
      m_imu_msg.linear_acceleration.z = a.Z() + distA(gen);
      m_imu_msg.linear_acceleration_covariance = {
        LINACC_NOISE_VAR, 0, 0,
        0, LINACC_NOISE_VAR, 0,
        0, 0, LINACC_NOISE_VAR
      };
      m_pubIMU.publish(m_imu_msg);

      m_encoder_msg.header.stamp    = ros::Time::now();
      m_encoder_msg.header.frame_id = "encoder";


      double yaw = rot.Yaw();
      // double angle_diff = speedYaw - yaw;
      // while(angle_diff > M_PI) angle_diff -= 2*M_PI;
      // while(angle_diff < -M_PI) angle_diff += 2*M_PI;
      // if (std::fabs(angle_diff) > 3*M_PI/4)
      // {
      //   speed *= -1;  // traveling backwards
      // }
      double forward_component = x_speed * std::cos(yaw) + y_speed * std::sin(yaw);
      const double threshold = 0.02;
      if (forward_component < -threshold) {
        speed = -speed;
      }

      std::normal_distribution<double> distE(0.0, ENCODER_SPEED_NOISE_STD);
      double noisy_speed = speed + distE(gen);
      encoder_buffer.emplace_back(now, noisy_speed);
      ros::Time prune_before = now - encoder_lag - ros::Duration(1.0);
      while (!encoder_buffer.empty() && encoder_buffer.front().first < prune_before) {
        encoder_buffer.pop_front();
      }
      ros::Time target = now - encoder_lag;
      double lagged_speed = noisy_speed;
      for (auto &entry : encoder_buffer) {
        if (entry.first <= target) {
          lagged_speed = entry.second;
        }
        else {
          break;
        }
      }
      m_encoder_msg.header.stamp = now;
      m_encoder_msg.header.frame_id = "encoder";
      m_encoder_msg.speed           = lagged_speed;
      m_pubEncoder.publish(m_encoder_msg);
    }

  } // namespace bno055
  GZ_REGISTER_MODEL_PLUGIN(bno055::BNO055)
}
