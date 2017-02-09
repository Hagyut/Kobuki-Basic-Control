#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ecl/threads.hpp>
#include <ecl/time.hpp>
#include <termios.h>

#include <kobuki_msgs/MotorPower.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define KeyCode_Right 67
#define KeyCode_Left 68
#define KeyCode_Up 65
#define KeyCode_Down 66
#define KeyCode_Space 32
#define KeyCode_Enable 101
#define KeyCode_Disable 100
#define KeyCode_Quit 113

#define linear_vel_step 0.1f
#define linear_vel_max 3.4f
#define angular_vel_step 0.1f
#define angular_vel_max 1.0f

ros::Publisher velocity_publisher;
ros::Publisher motor_power_publisher;

bool quit;
int key;
struct termios original_terminal_state;
bool power_status;
bool last_zero_vel_sent;

kobuki_msgs::MotorPower power_cmd;
geometry_msgs::Twist cmd;

ecl::Thread thr;

ecl::MilliSleep millisleep;

void init();
void spin();
void readKeyboradInput();
void processKeyInput(char key);

void disable();
void enable();
void incrementLinearVelocity();
void decrementLinearVelocity();
void incrementAngularVelocity();
void decrementAngularVelocity();
void resetVelocity();

int main (int argc, char **argv) {

    quit = false;
    key = 0;
    power_status = false;
    last_zero_vel_sent = true;
    tcgetattr(key, &original_terminal_state);

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    ros::init(argc, argv, "gted_kobuki_basic_control_node");
    ros::NodeHandle nh;
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    motor_power_publisher = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
    init();
    spin();
    
    tcsetattr(key, TCSANOW, &original_terminal_state);

    return 0;
}

void init() {
    bool connected = false;
    int count = 0;
    //ROS_INFO_STREAM("Subscribers: " << motor_power_publisher.getNumSubscribers());
    while (!connected) {
        int subsc_num = motor_power_publisher.getNumSubscribers();
        millisleep(500);
        ++count;
        if (count > 10) {
            break;
        }
        if (motor_power_publisher.getNumSubscribers() > 0) {
            connected = true;
        } else {
            
        }
        ROS_INFO_STREAM("Remaining tries: " << 10 - count);
    }
    if (count > 10) {
        ROS_INFO_STREAM("Cannot connect.");
        exit(-1);
    } else {
        ROS_INFO("Connected!");
    }

    thr.start(readKeyboradInput);
}

void spin() {
    ros::Rate loop_rate(10);
    int count = 0; 
    while (!quit && ros::ok()) {
        
        if ((cmd.linear.x  != 0.0) || (cmd.linear.y  != 0.0) || (cmd.linear.z  != 0.0) ||
            (cmd.angular.x != 0.0) || (cmd.angular.y != 0.0) || (cmd.angular.z != 0.0))
        {
            velocity_publisher.publish(cmd);
            last_zero_vel_sent = false;
        }
        else if (last_zero_vel_sent == false)
        {
            velocity_publisher.publish(cmd);
            last_zero_vel_sent = true;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    if (quit) {
        disable();
        puts("Terminate program.");
    }
    thr.join();
}

void readKeyboradInput() {

    struct termios raw;
    memcpy(&raw, &original_terminal_state, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;

    tcsetattr(key, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Forward/back arrows : linear velocity incr/decr.");
    puts("Right/left arrows : angular velocity incr/decr.");
    puts("Spacebar : reset linear/angular velocities.");
    puts("d : disable motors.");
    puts("e : enable motors.");
    puts("q : quit.");
    
    bool run = true;
    while (!quit && run) {
        char temp_c;
        if (read(key, &temp_c, 1) < 0) {
            perror("read char failed(): ");
            exit(-1);
        }
        processKeyInput(temp_c);
    }
}

void processKeyInput(char c) {
    switch(c) {
        case KeyCode_Right:
            decrementAngularVelocity();
        break;
        case KeyCode_Left:
            incrementAngularVelocity();
        break;
        case KeyCode_Up:
            incrementLinearVelocity();
        break;
        case KeyCode_Down:
            decrementLinearVelocity();
        break;
        case KeyCode_Space:
            resetVelocity();
        break;
        case KeyCode_Enable:
            enable();
        break;
        case KeyCode_Disable:
            disable();
        break;
        case KeyCode_Quit:
            quit = true;
        break;
        default:
        break;
    }
}


void disable()
{
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  velocity_publisher.publish(cmd);

  if (power_status)
  {
    ROS_INFO("KeyOp: die, die, die (disabling power to the device's motor system).");
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::OFF;
    motor_power_publisher.publish(power_cmd);
    power_status = false;
  }
  else
  {
    ROS_WARN("KeyOp: Motor system has already been powered down.");
  }
}

void enable()
{
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  velocity_publisher.publish(cmd);

  if (!power_status)
  {
    ROS_INFO("KeyOp: Enabling power to the device subsystem.");
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::ON;
    motor_power_publisher.publish(power_cmd);
    power_status = true;
  }
  else
  {
    ROS_WARN("KeyOp: Device has already been powered up.");
  }
}

void incrementLinearVelocity()
{
  if (power_status)
  {
    if (cmd.linear.x <= linear_vel_max)
    {
      cmd.linear.x += linear_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity incremented [" << cmd.linear.x << "|" << cmd.angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void decrementLinearVelocity()
{
  if (power_status)
  {
    if (cmd.linear.x >= -linear_vel_max)
    {
      cmd.linear.x -= linear_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity decremented [" << cmd.linear.x << "|" << cmd.angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void incrementAngularVelocity()
{
  if (power_status)
  {
    if (cmd.angular.z <= angular_vel_max)
    {
      cmd.angular.z += angular_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity incremented [" << cmd.linear.x << "|" << cmd.angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void decrementAngularVelocity()
{
  if (power_status)
  {
    if (cmd.angular.z >= -angular_vel_max)
    {
      cmd.angular.z -= angular_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity decremented [" << cmd.linear.x << "|" << cmd.angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void resetVelocity()
{
  if (power_status)
  {
    cmd.angular.z = 0.0;
    cmd.linear.x = 0.0;
    ROS_INFO_STREAM("KeyOp: reset linear/angular velocities.");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}