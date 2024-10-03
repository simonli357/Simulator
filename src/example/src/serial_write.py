import serial

def speed(f_velocity):
    return "#{0}:{1:.2f};;\r\n".format(1, f_velocity)

def steer(f_angle):
    return "#{0}:{1:.2f};;\r\n".format(2, f_angle)

def both(f_velocity,f_angle):
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(8, f_velocity, f_angle)

def main():
    serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200)

    # speed_msg = speed(10.0)
    # print("speed msg sent:", speed_msg)
    # serial_port.write(speed_msg.encode())

    # steer_msg = steer(10.0)
    # print("steer msg sent:", steer_msg)
    # serial_port.write(steer_msg.encode())

    both_msg = both(0.0, 0.0)
    print("both msg sent:", both_msg)
    serial_port.write(both_msg.encode())

    serial_port.close()

if __name__ == "__main__":
    main()
