#include <iostream>
#include <string>
#include <boost/asio.hpp>

using namespace std;

string speed(float f_velocity) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_velocity);
    strs << "#" << "1" << ":" << buff;
    return strs.str();
}

string steer(float f_angle) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_angle);
    strs << "#" << "2" << ":" << buff;
    return strs.str();
}

string both(float f_velocity, float f_angle) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity, f_angle);
    strs << "#" << "8" << ":" << buff;
    return strs.str();
}

int main() {
    boost::asio::io_service io;
    boost::asio::serial_port serial(io);

    serial.open("/dev/ttyACM0");
    serial.set_option(boost::asio::serial_port_base::baud_rate(19200));

    string speed_msg = speed(0.0);
    cout << "speed msg sent: " << speed_msg << endl;
    boost::asio::write(serial, boost::asio::buffer(speed_msg));

    string steer_msg = steer(0.0);
    cout << "steer msg sent: " << steer_msg << endl;
    boost::asio::write(serial, boost::asio::buffer(steer_msg));

    string both_msg = both(0.0, 0.0);
    cout << "both msg sent: " << both_msg << endl;
    boost::asio::write(serial, boost::asio::buffer(both_msg));

    serial.close();

    return 0;
}
