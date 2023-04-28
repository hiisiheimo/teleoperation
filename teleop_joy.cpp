#include "teleop_joy.hpp"
#include <stdio.h>
#include <termios.h>

TeleopJoyNode::TeleopJoyNode(int16_t max_torque)
    : max_torque(max_torque)
    , mtr_ctrl(0)
    , trn_rad(STEERING_MIDDLE_POSITION)
{}

void TeleopJoyNode::read_joystick_input(const sensor_msgs::Joy::ConstPtr& joy) {
    //update_turning_radius(joy);
    //update_motor_control(joy);
    std::cout << trn_rad << "  " << mtr_ctrl << std::endl;
}

void TeleopJoyNode::send_controls(ros::ServiceClient& client) const {
    atv_can::DriveService service_call;

    service_call.request.turning_radius = trn_rad;
    service_call.request.motor_control = mtr_ctrl;
    service_call.request.gear_ratio = 0;
    service_call.request.all_wheel_drive = 0;
    service_call.request.control_mode = true;  // Torque control (set to false for velocity
                                     // set point control.
    service_call.request.direction = true;
    if (client.call(service_call))
    {
        //std::cout << "sent-received" << service_call.response.status << std::endl;
        ROS_DEBUG("message sent");
    }
    else
    {
        //std::cout << "Failed to call service" << std::endl;
        ROS_ERROR("Failed to call service");
    }
}

void TeleopJoyNode::update_turning_radius(const sensor_msgs::Joy::ConstPtr& joy) {
    // joy->axes[0] is floating point number from -32768 to 32767
    trn_rad = 32767 + static_cast<uint16_t>(joy->axes[0]);
}

void TeleopJoyNode::update_motor_control(const sensor_msgs::Joy::ConstPtr& joy) {
    mtr_ctrl = static_cast<int16_t>(-max_torque*joy->axes[1] / JOYSTICK_INPUT_SCALE);
}

int TeleopJoyNode::getch() // it's copypasted from internet, hopefully it works
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  newt.c_cc[VMIN] = 0; 
  newt.c_cc[VTIME] = 0;      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle nh_("~"); //Private nodehandle
  ros::NodeHandle nh;
  int max_torque;
  if (nh_.getParam("max_torque", max_torque))
  {
    if (max_torque < 0 || max_torque > 100)
    {
      ROS_WARN("Torque value out of bounds, using default torque 50");
      max_torque = 50;
    }
    else
    {
      ROS_INFO("Setting maximum torque to %d", max_torque);
    }
  }
  else
  {
    ROS_WARN("Maximum torque not set, using default 50");
    max_torque = 50;
  }
  
  // Wait for the can_service to start
  ros::service::waitForService("can_service");

  ros::Rate loop_rate(10);
  TeleopJoyNode teleop_joy((int16_t)max_torque);

  ros::ServiceClient client(nh.serviceClient<atv_can::DriveService>("can_service"));

  ros::Subscriber subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoyNode::read_joystick_input, &teleop_joy);
  //signal(SIGINT,quit);
  //std::cout << "Entering while loop" << std::endl;
  bool speedTest = false;
  bool steeringTest = false;
  int64_t count = 0;

  while (ros::ok())
  {
    int c = getch();   // get keyboard input
    if (c == 'a')
    {
      std::cout << "You pressed a, starting speed test" << std::endl;
      speedTest = true;
    }
    if (c == 'b')
    {
      std::cout << "You pressed b, starting steering test" << std::endl;
      trn_rad = 0;
      count++;
      steeringTest = true;
    }
    if (c == 's')
    {
      std::cout << "You pressed s, stoping tests" << std::endl;
      trn_rad = STEERING_MIDDLE_POSITION;
      mtr_ctrl = 0;
      speedTest = false;
      steeringTest = false;
    }

    if(not count % 100 && speedTest) // 10 sec?
    {
      if(mtr_ctrl < 50)
      {
        mtr_ctrl += 10;
        ROS_INFO("Setting mtr_ctrl to %u", mtr_ctrl);
      }
      else
      {
        speedTest = false;
        mtr_ctrl = 0; // stopping motor
        ROS_INFO("Ending test and setting mtr_ctrl to %u", mtr_ctrl);
      }
    }

    if (not count % 100 && steeringTest)
    {
      else if(trn_rad < STEERING_MIDDLE_POSITION * 2)
      {
        trn_rad += (STEERING_MIDDLE_POSITION/5);
        trn_rad = (trn_rad > STEERING_MIDDLE_POSITION * 2) ?  STEERING_MIDDLE_POSITION * 2 : trn_rad; // maximun is STEERING_MIDDLE_POSITION * 2
        ROS_INFO("Setting trn_rad to %d", trn_rad);
      }
      else
      {
          steeringTest = false;
          trn_rad = STEERING_MIDDLE_POSITION;
          mtr_ctrl = 0; //make sure the motor is off
          ROS_INFO("Ending test and setting trn_rad to %d", trn_rad);
      }
    }
    teleop_joy.send_controls(client);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return(0);
}
