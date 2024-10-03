#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace boost::asio;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_imu_reader_node");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/car1/data", 10);

    io_service io;
    serial_port serial(io);

    serial.open("/dev/ttyACM0");
    serial.set_option(serial_port_base::baud_rate(115200));

    char data[256]; // Buffer to store data
    size_t length = 0;
    string buffer; // Buffer to accumulate the received data

    while (ros::ok()) {
        // Read until '@7' is found
        do {
            length = serial.read_some(mutable_buffer(data, 256)); // Read data from serial port
            buffer.append(data, length);
        } while (buffer.find("@7") == string::npos);

        // Find the end of line
        size_t end_pos = buffer.find('\n');
        if (end_pos != string::npos) {
            string line = buffer.substr(0, end_pos); // Extract the line
            buffer.erase(0, end_pos + 1); // Remove the processed part from the buffer

            if (line.find("@7") != string::npos) {
                // cout << "Received data: " << line << endl; // Print received data
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "imu_frame";

                // Extract prefix and ignore '@' and ':' characters
                string prefix;
                size_t pos = line.find(':');
                if (pos != string::npos && pos + 1 < line.length()) {
                    prefix = line.substr(pos + 1);
                } else {
                    cerr << "Error: Failed to extract prefix from the string." << endl;
                    continue;
                }

                // Extract substrings between ';' characters
                std::string roll_str, pitch_str, yaw_str, accelx_str, accely_str, accelz_str, gyrox_str, gyroy_str, gyroz_str;
                size_t end_pos = pos;
                pos++;
                for (int i = 0; i < 9; ++i) {
                    end_pos = line.find(';', pos);
                    if (end_pos != std::string::npos) {
                        switch (i) {
                            case 0:
                                roll_str = line.substr(pos, end_pos - pos);
                                break;
                            case 1:
                                pitch_str = line.substr(pos, end_pos - pos);
                                break;
                            case 2:
                                yaw_str = line.substr(pos, end_pos - pos);
                                break;
                            case 3:
                                accelx_str = line.substr(pos, end_pos - pos);
                                break;
                            case 4:
                                accely_str = line.substr(pos, end_pos - pos);
                                break;
                            case 5:
                                accelz_str = line.substr(pos, end_pos - pos);
                                break;
                            case 6:
                                gyrox_str = line.substr(pos, end_pos - pos);
                                break;
                            case 7:
                                gyroy_str = line.substr(pos, end_pos - pos);
                                break;
                            case 8:
                                gyroz_str = line.substr(pos, end_pos - pos);
                                break;
                        }
                        pos = end_pos + 1;
                    } else {
                        std::cerr << "Error: Failed to find delimiter in the string." << std::endl;
                        continue;
                    }
                }

                // cout << "Roll: " << roll_str << endl;
                // cout << "Pitch: " << pitch_str << endl;
                // cout << "Yaw: " << yaw_str << endl;
                // cout << "Accel X: " << accelx_str << endl;
                // cout << "Accel Y: " << accely_str << endl;
                // cout << "Accel Z: " << accelz_str << endl;
                // cout << "Gyro X: " << gyrox_str << endl;
                // cout << "Gyro Y: " << gyroy_str << endl;
                // cout << "Gyro Z: " << gyroz_str << endl;

                // Convert substrings to floating-point numbers
                double roll = stod(roll_str);
                double pitch = stod(pitch_str);
                double yaw = stod(yaw_str);
                double accelx = stod(accelx_str);
                double accely = stod(accely_str);
                double accelz = stod(accelz_str);
                double gyrox = stod(gyrox_str);
                double gyroy = stod(gyroy_str);
                double gyroz = stod(gyroz_str);

                if (abs(accelx)>0.1){
                    cout << "AX: " << accelx_str << endl;
                }
                if (abs(accely)>0.1){
                    cout << "AY: " << accely_str << endl;
                }

                // Output the extracted values
                // cout << "Roll: " << roll << endl;
                // cout << "Pitch: " << pitch << endl;
                // cout << "Yaw: " << yaw << endl;
                // cout << "Accel X: " << accelx << endl;
                // cout << "Accel Y: " << accely << endl;
                // cout << "Accel Z: " << accelz << endl;
                // cout << "Gyro X: " << gyrox << endl;
                // cout << "Gyro Y: " << gyroy << endl;
                // cout << "Gyro Z: " << gyroz << endl;

                // Convert Euler angles to quaternion
                tf2::Quaternion q;
                q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
                imu_msg.orientation.x = q.x();
                imu_msg.orientation.y = q.y();
                imu_msg.orientation.z = q.z();
                imu_msg.orientation.w = q.w();

                // Set linear acceleration
                imu_msg.linear_acceleration.x = accelx;
                imu_msg.linear_acceleration.y = accely;
                imu_msg.linear_acceleration.z = accelz;

                imu_msg.angular_velocity.x = gyrox*2*M_PI;
                imu_msg.angular_velocity.y = gyroy*2*M_PI;
                imu_msg.angular_velocity.z = gyroz*2*M_PI;

                imu_pub.publish(imu_msg); // Publish the IMU message
            }
        }

        ros::spinOnce();
    }

    return 0;
}
