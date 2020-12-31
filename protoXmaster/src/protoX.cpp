/** protoX.cpp
 * write by Mr.Kraiwit Trisaksri
 * at coXsys robotics.
*/
#include <iostream>                          // include iostream.
#include <sstream>                           // include sstream.
#include <exception>                         // include exeption.
#include <chrono>                            // include chrono for use time.
#include <thread>                            // include thread.
#include "register_table.h"                  // include register_table.
#include "Proto_X_master/Proto_X_master.cpp" // include protoXmaster.
#include "protoX_utils.h"                    // include protoXmaster utility.
////////////////////////////////////////////////
#include "ros/ros.h" // include ROS and some message.
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/ByteMultiArray.h"
//##################################
//######### USER SETTING ###########
//##################################
REGISTER_TABLE regis;                    // create register_table Obj.
ProtoXmaster<REGISTER_TABLE> px(&regis); // create protoXmaster Obj.
std::string port = "/dev/ttyUSB0";       // default device port.

//##### USER DEFINE #####
#define PROTOXMASTER_VERSION "2.0.0" // protoXmaster version.

#define SIZE_MESSAGE_BUFFER 100 // size of message buffer.
#define wheel_radius 0.085      // unit: meter.
#define dimension_long 0.235    // unit: meter.

#define PERIOD_ROS_UPDATE_STATUS 1000 // unit: millisecond  // use for publish data to ROS.
#define PERIOD_ROS_UPDATE_SENSORS 20  // unit: millisecond  // use for publish data to ROS.
#define PERIOD_GET_STATUS 1000        // unit: millisecond  // use for setting protoXslave.
#define PERIOD_GET_SENSORS 20         // unit: millisecond  // use for setting protoXslave.
#define BUZZER_TIMEOUT 5000           // unit: millisecond.
#define TIME_SYNC_SERIAL 200          // unit: millisecond.
#define MAX_RESYNC_TIME 2             // NOTE:: MAX_RESYNC_TIME * TIME_SYNC_SRIAL must be less than 1000.

/////////////////////////// Global Variable ///////////////////////////////
using namespace std::chrono;                    // use namespace.
auto Main_Clock = high_resolution_clock::now(); // Obj. for Main timer.
uint64_t Main_time = 0;                         // Main timer.
uint64_t time_stamp_reset_buzzer = 0;           // time stamp for reset buzzer.
uint64_t time_stamp_sync_serial = 0;            // time stamp for check sync with protoXslave via serial.
uint8_t resync_time = 0;                        // resync protoX version time.
bool is_exit = false;                           // boolean for exit this program.
bool ready_ros_update_sensors = false;          // boolean for send data to ROS(sensors without bumper).
bool ready_ros_update_status = false;           // boolean for send data to ROS(status).

bool check_on_melody = false;     // boolean for allow to play buzzer.
bool ready_report_bumper = false; // boolean for send data to ROS(bumper only).
bool bumper_prev_state = false;   // boolean for save previous bumper state.
bool not_first_sync = false;      // boolean for first time sync with protoXslave.
////////////////////////////// Prototype Function ///////////////////////////////////
///// Function Callback /////
void CMD_Actuator_Callback(const std_msgs::UInt8 &input_actuator_status); // Callback Function to command Actuator state via protoXslave.
void CMD_Robot_Callback(const geometry_msgs::Twist &input_cmd_robot);     // Callback Function to command Robot to driving via protoXslave.
void CMD_Light_Callback(const std_msgs::UInt8 &input_cmd_light);          // Callback Function to on/off Lights state via protoXslave.
void Buzzer_Melody_Callback(const std_msgs::UInt8 &input_melody);         // Callback Function to play/off buzzer melody.

void Neopixel_mode_Callback(const std_msgs::UInt8 &input_mode);                // Callback Function to set mode of Neopixel via protoXslave.
void Neopixel_rgb_Callback(const std_msgs::UInt8MultiArray &input_rgb);        // Callback Function to set rgb(red green blue) of Neopixel via protoXslave.
void Neopixel_mstime_Callback(const std_msgs::UInt16MultiArray &input_mstime); // Callback Function to set mstime(millisecond) of Neopixel via protoXslave.
void Neopixel_num_Callback(const std_msgs::UInt16MultiArray &input_num);       // Callback Function to set num(number of led) of Neopixel via protoXslave.
void Neopixel_step_Callback(const std_msgs::UInt16 &input_step);               // Callback Function to set step(number of led) of Neopixel via protoXslave.

void sig_to_exception(int s) // Function for detect Ctrl-C KeyBoardInterrupt.
{
    throw InterruptException(s);
}
///// In Loop Function /////
void Main_timer_Update();    // Function to read current of time and update to Main_time.
void ROS_Update();           // Function to publish data to ROS.
void protoX_auto_read();     // Function for read "protoX auto return" from protoXslave.
void check_buzzer_timeout(); // Function to check buzzer command timeout.

///// Normal Function /////
void Display_Robot_Info();       // Function to read robot infomation and check sync the version of protoXmaster and protoXslave.
void Setup_ProtoX_auto_return(); // Function to set protoXslave to start protoX auto return (use PERIOD_GET_STATUS and PERIOD_GET_SENSORS).
void Reset_ProtoX_auto_return(); // Function to set protoXslave to stop protoX auto return.

//// Status section
//// create ROS message for status section.
std_msgs::UInt8 device_status;
std_msgs::UInt8 powerboard_status;
std_msgs::UInt8 battery_status;
std_msgs::UInt8 actuator_status;
std_msgs::UInt8 powerboard_errorcode;
std_msgs::UInt8 mcu_errorcode;

//// Sensors section
//// create ROS message for sensors section.
std_msgs::ByteMultiArray bumper_status;
std_msgs::UInt16MultiArray usonic;
std_msgs::UInt8 batt_percent;      // 0 - 100 %
std_msgs::Float32 sens_voltage;    // volt
std_msgs::Float32 sens_current;    // amp
std_msgs::Float32 sens_celsius;    // degree celsius

//// Odometry
//// create ROS message for return feedback velocity of robot to ROS.
geometry_msgs::Twist feedback_vel;

/////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    //// for Ctrl+C (KeyboardInterrupt)
    struct sigaction KeyBoardInterruptHandler;
    KeyBoardInterruptHandler.sa_handler = sig_to_exception;
    sigemptyset(&KeyBoardInterruptHandler.sa_mask);
    KeyBoardInterruptHandler.sa_flags = 0;

    if (argc > 2) // check argument of protoXmaster protoX
    {
        std::cout << col_yellow << "[Warning Argument][Usage]: $ rosrun protoXmaster protoX" << col_yellow << " [USBport]" << col_reset << std::endl;
        std::cout << col_yellow << "[Example run]: $ rosrun protoXmaster protoX /dev/ttyUSB0" << col_reset << std::endl
                  << col_yellow << "if don't fill USBport default = /dev/ttyUSB0" << col_reset << std::endl;
        port = argv[1]; // set argv[1] is serial port to connect with protoXslave .
        std::cout << col_yellow << std::endl
                  << ">> Use port " << argv[1] << col_reset << std::endl;
    }
    else
    {
        if (argc == 2)
            port = argv[1]; // set argv[1] is serial port to connect with protoXslave .
        else
            std::cout << col_yellow << ">> Use default port /dev/ttyUSB0" << col_reset << std::endl;
    }
    px.setPort(port, 115200); // setup serial port and baudrate.

    //////////////////// ROS Setup /////////////////////
    ros::init(argc, argv, "proto_x_master"); // initial ROS.
    ros::NodeHandle protoXmaster_node;       // create NodeHandle.

    //////////////////// Setup [ROS] Publisher ////////////////
    ros::Publisher pub_device_stats = protoXmaster_node.advertise<std_msgs::UInt8>("/device_status", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_powerboard_status = protoXmaster_node.advertise<std_msgs::UInt8>("/powerboard_status", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_battery_status = protoXmaster_node.advertise<std_msgs::UInt8>("/battery_status", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_actuator_status = protoXmaster_node.advertise<std_msgs::UInt8>("/actuator_status", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_powerboard_errorcode = protoXmaster_node.advertise<std_msgs::UInt8>("/powerboard_errorcode", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_mcu_errorcode = protoXmaster_node.advertise<std_msgs::UInt8>("/mcu_errorcode", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_batt_percent = protoXmaster_node.advertise<std_msgs::UInt8>("/battery_percent", SIZE_MESSAGE_BUFFER); // 0 - 100 %
    ros::Publisher pub_sens_voltage = protoXmaster_node.advertise<std_msgs::Float32>("/sens_voltage", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_sens_current = protoXmaster_node.advertise<std_msgs::Float32>("/sens_current", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_sens_celsius = protoXmaster_node.advertise<std_msgs::Float32>("/sens_celsius", SIZE_MESSAGE_BUFFER);

    ros::Publisher pub_bumper_status = protoXmaster_node.advertise<std_msgs::ByteMultiArray>("/bumper_data", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_usonic = protoXmaster_node.advertise<std_msgs::UInt16MultiArray>("/ultrasonic_data", SIZE_MESSAGE_BUFFER);
    ros::Publisher pub_feedback_vel = protoXmaster_node.advertise<geometry_msgs::Twist>("/feedback_vel", SIZE_MESSAGE_BUFFER);

    ///////////////// Setup [ROS] Subscriber ///////////////
    ros::Subscriber sub_actuator_status = protoXmaster_node.subscribe("/actuator_status", SIZE_MESSAGE_BUFFER, CMD_Actuator_Callback);

    ros::Subscriber sub_cmd_robot = protoXmaster_node.subscribe("/coconut_vel", SIZE_MESSAGE_BUFFER, CMD_Robot_Callback);
    ros::Subscriber sub_cmd_light = protoXmaster_node.subscribe("/cmd_light", SIZE_MESSAGE_BUFFER, CMD_Light_Callback);
    ros::Subscriber sub_buzzer_melody = protoXmaster_node.subscribe("/buzzer_melody", SIZE_MESSAGE_BUFFER, Buzzer_Melody_Callback);
    ros::Subscriber sub_neopixel_mode = protoXmaster_node.subscribe("/neopixel_mode", SIZE_MESSAGE_BUFFER, Neopixel_mode_Callback);
    ros::Subscriber sub_neopixel_rgb = protoXmaster_node.subscribe("/neopixel_rgb", SIZE_MESSAGE_BUFFER, Neopixel_rgb_Callback);
    ros::Subscriber sub_neopixel_mstime = protoXmaster_node.subscribe("/neopixel_time_ms", SIZE_MESSAGE_BUFFER, Neopixel_mstime_Callback);
    ros::Subscriber sub_neopixel_num = protoXmaster_node.subscribe("/neopixel_num", SIZE_MESSAGE_BUFFER, Neopixel_num_Callback);
    ros::Subscriber sub_neopixel_step = protoXmaster_node.subscribe("/neopixel_step", SIZE_MESSAGE_BUFFER, Neopixel_step_Callback);

    sigaction(SIGINT, &KeyBoardInterruptHandler, NULL); //// for Ctrl+C (KeyboardInterrupt).
    //// try to connect to serial port.
    try
    {
        px.begin_until_connected(); // protoXmaster connect to serial port.
    }
    catch (InterruptException &e) // catch Ctrl+C (KeyboardInterrupt).
    {
        std::cout << std::endl
                  << col_magenta << ">>> Exit with KeyboardInterrupt." << col_reset << std::endl;
        return EXIT_SUCCESS;
    }

    Display_Robot_Info(); //// Display and check sync Robot version.
    if (is_exit)
        return EXIT_SUCCESS;

    std::thread protoX_read_thread(protoX_auto_read); // start thread to receive auto return from protoXslave.
    Setup_ProtoX_auto_return();                       // setup auto return.
    bool work = true;                                 // boolean to check is the flow work?
    while (work)                                      // check is the flow work?
    {
        try
        {
            ros::spinOnce(); // update and sync with ROS.
            if (ros::ok())   // check is ROS ok?
            {
                Main_timer_Update();         // update Main_time.
                ROS_Update();                // update data from protoXmaster to ROS.
                check_buzzer_timeout();      // check buzzer command timeout.
                if (ready_ros_update_status) // check for publish status section.
                {
                    ready_ros_update_status = false;                        // reset boolean.
                    pub_device_stats.publish(device_status);                // publish device_status.
                    pub_powerboard_status.publish(powerboard_status);       // publish powerboard_status.
                    pub_battery_status.publish(battery_status);             // publish battery_status.
                    pub_actuator_status.publish(actuator_status);           // publish actuator_status.
                    pub_powerboard_errorcode.publish(powerboard_errorcode); // publish powerboard_errorcode.
                    pub_mcu_errorcode.publish(mcu_errorcode);               // publish mcu_errorcode.
                    pub_batt_percent.publish(batt_percent);                 // publish batt_percent.
                    pub_sens_voltage.publish(sens_voltage);                 // publish sens_voltage.
                    pub_sens_current.publish(sens_current);                 // publish sens_current.
                    pub_sens_celsius.publish(sens_celsius);                 // publish sens_celcius.
                    px.send_ping_no_return();                               //// need : ping to tell protoXslave is connected with protoXmaster
                }
                if (ready_ros_update_sensors) // check for publish sensors section.
                {
                    ready_ros_update_sensors = false;       // reset boolean.
                    pub_feedback_vel.publish(feedback_vel); // publish feedback_vel.
                    pub_usonic.publish(usonic);             // publish usonic.
                }
                if (ready_report_bumper) // check for publish bumper status.
                {
                    ready_report_bumper = false;              // reset boolean.
                    pub_bumper_status.publish(bumper_status); // publish bumper_status.
                }
                if (Main_time - time_stamp_sync_serial >= TIME_SYNC_SERIAL) // check is thread read auto return from protoXslave working?
                {
                    if (resync_time >= MAX_RESYNC_TIME) // check resync time.
                    {
                        std::cout << col_red << "ControlBoard lost sync or USB port disconnect." << col_reset << std::endl;
                        is_exit = true; // set to exit the program.
                        work = false;   // set to exit the program.
                    }
                    else
                    {
                        if (not_first_sync) // check not_first_sync
                        {
                            std::cout << col_yellow << "Resync.." << col_reset << std::endl;
                            resync_time++;
                            Setup_ProtoX_auto_return(); // setup auto return to protoXslave.
                        }
                        time_stamp_sync_serial = Main_time; // reset time_stamp_sync_serial.
                        not_first_sync = true;              // set boolean.
                    }
                }
            }
            else
            {
                std::cout << col_red << "ROS::not ok or USB port loose." << col_reset << std::endl;
                is_exit = true; // set to exit the program.
                break;
            }
        }
        catch (const InterruptException &e) // catch Ctrl+C (KeyboardInterrupt).
        {
            is_exit = true; // set to exit the program.
            std::cout << std::endl
                      << col_magenta << ">>> Exit by KeyboardInterrupt." << col_reset << std::endl;
            work = false;
        }
        catch (serial::SerialException &e) // catch exception of Serial.
        {
            is_exit = true; // set to exit the program.
            std::cout << std::endl
                      << col_magenta << ">>> Exit by USB Serial lost the conenction." << col_reset << std::endl;
            work = false;
        }
        catch (std::exception &e) // catch exception.
        {
            is_exit = true; // set to exit the program.
            std::cout << std::endl
                      << col_magenta << e.what() << std::endl;
            work = false;
        }
        catch (...)
        {
            is_exit = true; // set to exit the program.
            std::cout << std::endl
                      << col_magenta << "Unexpected Error." << col_reset << std::endl;
            work = false;
        }
    }
    std::cout << "--------------------------------------------" << col_reset << std::endl;
    protoX_read_thread.join();  // wait protoX_read_thread to end the process.
    Reset_ProtoX_auto_return(); // reset auto return to protoXslave.
    return EXIT_SUCCESS;
}

/////////////////// Callback Function ///////////////////
void CMD_Actuator_Callback(const std_msgs::UInt8 &input_actuator_status)
{
    /** Actuator_status [All device use 24V+ (Motor)]
     * 		0				|	OFF
     * 		1				|	ON
     * 		2				|	INIT
     * 		3				|	push Emergency button
     * 		4				|	Application command OFF
     * 		5				|	Application command ON
     * 		50				|	MotorDrive OVER-CURRENT
     * 		51				|	MotorDrive OVER-VOLTAGE
     * 		52				|	MotorDrive Encoder Failure
     * 		53				|	MotorDrive OVER HEAT
     * 		54				|	MotorDrive UNDER-VOLTAGE
     * 		55				|	MotorDrive OVER LOAD
     * 		100				|	Reset Alarm
     * */
    //// NOTE: we allow to send only 4, 5, 100.
    if (input_actuator_status.data == 4 || input_actuator_status.data == 5 || input_actuator_status.data == 100) // check actuator_status from ros
    {
        // protoXmaster send data to write register_table.
        px.send_write_data(px.get_address(&(regis.status.actuator_status)), (uint8_t *)&(input_actuator_status.data), sizeof(regis.status.actuator_status));
    }
}
void CMD_Robot_Callback(const geometry_msgs::Twist &input_cmd_robot)
{
    regis.command.cmd_robot.linear_vel_x = input_cmd_robot.linear.x;       // save linear_vel_x to protoXmaster table.
    regis.command.cmd_robot.angular_vel_theta = input_cmd_robot.angular.z; // save angular_vel_theta to protoXmaster table.
    // protoXmaster send data to write register_table.
    px.send_write_data(px.get_address(&(regis.command.cmd_robot)), (uint8_t *)&(regis.command.cmd_robot), sizeof(regis.command.cmd_robot));
}
void CMD_Light_Callback(const std_msgs::UInt8 &input_cmd_light)
{
    regis.command.light_on = input_cmd_light.data; // save light_on to protoXmaster table.
    // protoXmaster send data to write register_table.
    px.send_write_data(px.get_address(&(regis.command.light_on)), (uint8_t *)&(regis.command.light_on), sizeof(regis.command.light_on));
}
void Buzzer_Melody_Callback(const std_msgs::UInt8 &input_melody)
{
    check_on_melody = true;                               // on melody.
    if (input_melody.data > 0 && input_melody.data <= 65) // check input.
    {
        regis.command.buzzer_melody = input_melody.data;
        time_stamp_reset_buzzer = Main_time; // start timer for on buzzer.
    }
    else
        regis.command.buzzer_melody = 0; // reset melody to play.
    // protoXmaster send data to write register_table.
    px.send_write_data(px.get_address(&(regis.command.buzzer_melody)), (uint8_t *)&(regis.command.buzzer_melody), sizeof(regis.command.buzzer_melody));
}

void Neopixel_mode_Callback(const std_msgs::UInt8 &input_mode)
{
    regis.command.neopixel.mode = input_mode.data; // save mode to protoXmaster table.
    // protoXmaster send data to write register_table.
    px.send_write_data(px.get_address(&(regis.command.neopixel)), (uint8_t *)&(regis.command.neopixel), sizeof(regis.command.neopixel));
}
void Neopixel_rgb_Callback(const std_msgs::UInt8MultiArray &input_rgb)
{
    //// save all rgb to protoXmaster table.
    if (input_rgb.data.size() > 0)
    {
        regis.command.neopixel.r[0] = input_rgb.data[0];
        regis.command.neopixel.g[0] = input_rgb.data[1];
        regis.command.neopixel.b[0] = input_rgb.data[2];
    }
    if (input_rgb.data.size() > 3)
    {
        regis.command.neopixel.r[1] = input_rgb.data[3];
        regis.command.neopixel.g[1] = input_rgb.data[4];
        regis.command.neopixel.b[1] = input_rgb.data[5];
    }
    if (input_rgb.data.size() > 6)
    {
        regis.command.neopixel.r[2] = input_rgb.data[6];
        regis.command.neopixel.g[2] = input_rgb.data[7];
        regis.command.neopixel.b[2] = input_rgb.data[8];
    }
}
void Neopixel_mstime_Callback(const std_msgs::UInt16MultiArray &input_mstime)
{
    if (input_mstime.data.size() <= 3)
        for (uint8_t i = 0; i < input_mstime.data.size(); i++)
            //// save all ms_time to protoXmaster table.
            regis.command.neopixel.ms_time[i] = input_mstime.data[i];
}
void Neopixel_num_Callback(const std_msgs::UInt16MultiArray &input_num)
{
    if (input_num.data.size() <= 3)
        for (uint8_t i = 0; i < input_num.data.size(); i++)
            //// save all num to protoXmaster table.
            regis.command.neopixel.num[i] = input_num.data[i];
}
void Neopixel_step_Callback(const std_msgs::UInt16 &input_step)
{
    //// save step to protoXmaster table.
    regis.command.neopixel.step = input_step.data;
}

////////////////// In Loop Function ///////////////
inline void Main_timer_Update()
{
    Main_Clock = high_resolution_clock::now();                                      // time stamp
    Main_time = duration_cast<milliseconds>(Main_Clock.time_since_epoch()).count(); // Get time in milliseconds
}

void ROS_Update()
{
    static uint64_t time_stamp_status = 0;
    static uint64_t time_stamp_sensors = 0;
    if (Main_time - time_stamp_sensors >= PERIOD_ROS_UPDATE_SENSORS)
    {
        time_stamp_sensors = Main_time; // reset time_stamp_sensors.

        feedback_vel.linear.x = regis.sens.feedback_vel.linear_vel_x;       // save linear_vel_x from protoXslave to ROS message.
        feedback_vel.angular.z = regis.sens.feedback_vel.angular_vel_theta; // save angular_vel_theta from protoXslave to ROS message.

        bumper_status.data.clear();                                                 // clear all bumper_status data.
        bumper_status.data.push_back((regis.sens.bumper_status & 0b00000001) || 0); // convert Bumper[0] data to 0|1 and save data to ROS message.
        bumper_status.data.push_back((regis.sens.bumper_status & 0b00000010) || 0); // convert Bumper[1] data to 0|1 and save data to ROS message.
        bumper_status.data.push_back((regis.sens.bumper_status & 0b00000100) || 0); // convert Bumper[2] data to 0|1 and save data to ROS message.
        bumper_status.data.push_back((regis.sens.bumper_status & 0b00001000) || 0); // convert Bumper[3] data to 0|1 and save data to ROS message.
        bumper_status.data.push_back((regis.sens.bumper_status & 0b00010000) || 0); // convert Bumper[4] data to 0|1 and save data to ROS message.
        bumper_status.data.push_back((regis.sens.bumper_status & 0b00100000) || 0); // convert Bumper[5] data to 0|1 and save data to ROS message.
        //// if push some Bumper
        if ((regis.sens.bumper_status & 0b00111111) != 0)
        {
            if (bumper_prev_state == false)
            {
                bumper_prev_state = true;   // save bumper previous state.
                ready_report_bumper = true; // set boolean to publish bumper_status.
            }
        }
        else //// Bumper Safe
        {
            if (bumper_prev_state == true)
            {
                bumper_prev_state = false;  // save bumper previous state.
                ready_report_bumper = true; // set boolean to publish bumper_status.
            }
        }
        usonic.data.clear();                          // clear all data in usonic.
        usonic.data.push_back(regis.sens.ult.dist1);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist2);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist3);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist4);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist5);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist6);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist7);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist8);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist9);  // save data to ROS message.
        usonic.data.push_back(regis.sens.ult.dist10); // save data to ROS message.
        ready_ros_update_sensors = true;              // set boolean to publish sensors.
    }
    if (Main_time - time_stamp_status >= PERIOD_ROS_UPDATE_STATUS)
    {
        time_stamp_status = Main_time;                                 // reset time_stamp_status.
        device_status.data = regis.status.device_status;               // save data to ROS message.
        powerboard_status.data = regis.status.powerboard_status;       // save data to ROS message.
        battery_status.data = regis.status.battery_status;             // save data to ROS message.
        actuator_status.data = regis.status.actuator_status;           // save data to ROS message.
        powerboard_errorcode.data = regis.status.powerboard_errorcode; // save data to ROS message.
        mcu_errorcode.data = regis.status.mcu_errorcode;               // save data to ROS message.

        batt_percent.data = regis.status.battery_percent;    // save data to ROS message.
        sens_voltage.data = regis.status.sens_voltage;       // save data to ROS message.
        sens_current.data = regis.status.sens_current;       // save data to ROS message.
        sens_celsius.data = regis.status.sens_celsius;       // save data to ROS message.
        ready_ros_update_status = true;                      // set boolean to publish status.
    }
}

void check_buzzer_timeout()
{
    if (check_on_melody) // check is melody on?
    {
        if (Main_time - time_stamp_reset_buzzer >= BUZZER_TIMEOUT) // check is Timeout?
        {
            regis.command.buzzer_melody = 0; // reset melody.
            // send data to protoXslave to silence buzzer.
            px.send_write_data(px.get_address(&(regis.command.buzzer_melody)), (uint8_t *)&(regis.command.buzzer_melody), sizeof(regis.command.buzzer_melody));
            check_on_melody = false; // reset boolean.
        }
    }
}

void protoX_auto_read()
{
    while (1)
    {
        if (!is_exit) // check is exit the program?
        {
            if (px._rx_ready) // check is protoXmaster receive ready?
            {
                time_stamp_sync_serial = Main_time; // reset timer to check is receive auto return.
                px.get_auto_return_toTable();       // get auto return data from protoXslave to protoXmaster table.
            }
        }
        else // exit the program.
        {
            break;
        }
    }
}

/////////////// Normal Function ///////////////
void Display_Robot_Info()
{
    std::string s = PROTOXMASTER_VERSION; // get PROTOXMASTER_VERSION from global variable.
    uint8_t my_version[3];                // variable use for save the protoXmaster version.
    uint8_t index = 0;
    size_t pos = 0;
    std::string token;
    // erase "." in PROTOXMASTER_VERSION.
    while ((pos = s.find(".")) != std::string::npos)
    {
        token = s.substr(0, pos);
        my_version[index++] = stoi(token);
        s.erase(0, pos + 1);
    }
    my_version[index] = stoi(s); // convert PROTOXMASTER_VERSION to number of version.

    // read data from infomation section from protoXslave.
    px.send_read_data_toTable(px.get_address(&(regis.info)), sizeof(regis.info));
    // calculate robot firmware version.
    uint16_t integer_version = regis.info.firmware_version / 100;
    uint8_t point2_version = regis.info.firmware_version % 10;
    uint8_t point1_version = ((regis.info.firmware_version % 100) - point2_version) / 10;
    //// Show data on the terminal that run this program.
    std::cout << col_blue << "#########################################################" << std::endl;
    std::cout << "################     CoXSys Robotics     ################" << std::endl;
    std::cout << "#########################################################" << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "================      Coconut Robot      ================" << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "protoXmaster Version   : " << PROTOXMASTER_VERSION << std::endl;
    std::cout << std::dec << "Robot Firmware Version : " << 0 + integer_version << "." << 0 + point1_version
              << "." << 0 + point2_version << std::endl;
    std::cout << "Period Control Time    : " << 0 + regis.info.control_time_period << " millisecond(s)" << std::endl
              << col_reset << std::endl;

    for (uint8_t i = 0; i <= 1; i++)
    {
        // robot firmware version not match with protoXmaster version.
        if ((integer_version != my_version[0]) || (point1_version != my_version[1]) || (point2_version != my_version[2]))
        {
            Reset_ProtoX_auto_return(); // reset auto return from protoXslave.
            if (i == 1)
            {
                std::cout << col_red << "[Error] Incorrect Version." << std::endl;
                is_exit = true; // set to exit the program.
            }
            else
            {
                // read data in infomation section from protoXslave.
                px.send_read_data_toTable(px.get_address(&(regis.info)), sizeof(regis.info));
                // calculate robot firmware version.
                integer_version = regis.info.firmware_version / 100;
                point2_version = regis.info.firmware_version % 10;
                point1_version = (regis.info.firmware_version % 100) - point2_version;
                std::cout << "-> " << col_yellow << "Sync Version.." << std::endl;
            }
            usleep(200000);
        }
        // correct version.
        else
        {
            is_exit = false;
            std::cout << "-> " << col_green << "Correct Version." << col_reset << std::endl;
            break;
        }
        // show the robot firmware version.
        std::cout << std::dec << "Robot Firmware Version : " << 0 + integer_version << "." << 0 + point1_version
                  << "." << 0 + point2_version << col_reset << std::endl
                  << std::endl;
    }
}

void Setup_ProtoX_auto_return()
{
    px.send_ping_no_return(); // send ping protocol to protoXslave.
    regis.status.update_period = PERIOD_GET_STATUS;
    // set update_period of status in protoXslave.
    px.send_write_data(px.get_address(&(regis.status.update_period)), (uint8_t *)&(regis.status.update_period), sizeof(regis.status.update_period));
    regis.sens.update_period = PERIOD_GET_SENSORS;
    // set update_period of sensors in protoXslave.
    px.send_write_data(px.get_address(&(regis.sens.update_period)), (uint8_t *)&(regis.sens.update_period), sizeof(regis.sens.update_period));
}

void Reset_ProtoX_auto_return()
{
    regis.status.update_period = 0;
    // set 0 to update_period of status in protoXslave.
    px.send_write_data(px.get_address(&(regis.status.update_period)), (uint8_t *)&(regis.status.update_period), sizeof(regis.status.update_period));
    regis.sens.update_period = 0;
    // set 0 to update_period of sensors in protoXslave.
    px.send_write_data(px.get_address(&(regis.sens.update_period)), (uint8_t *)&(regis.sens.update_period), sizeof(regis.sens.update_period));
}