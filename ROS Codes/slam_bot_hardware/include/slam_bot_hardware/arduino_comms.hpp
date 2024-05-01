#ifndef SLAM_BOT_HARDWARE__ARDUINO_HARDWARE_HPP
#define SLAM_BOT_HARDWARE__ARDUINO_HARDWARE_HPP

namespace slam_bot_hardware{
    class ArduinoComms{
        public:
            void initialize();

            void handshake();

            void update_motors(int left_speed, int right_speed);

            void update_servos(int servo_angles[4]);

            void read_motor_speeds(int &left_speed, int &right_speed);

            void ensure_get_send();

            void stop();
        private:
            float left_speed_, right_speed_, servo_angles_[4];

    };
}

#endif