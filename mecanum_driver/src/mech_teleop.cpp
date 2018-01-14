#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

class InvKinematics
{
public:
  InvKinematics(ros::NodeHandle nhand){
    nh_ = nhand;
    vel_pub_ = nh_.advertise<sensor_msgs::JointState>("wheels/js", 1);
  }
  void update(const sensor_msgs::Joy::ConstPtr& msg){
    float x = -1*(msg->axes[0]);
    float y = msg->axes[1];
    float z = -1*(msg->axes[2]);
    sensor_msgs::JointState js_;


    js_.velocity.push_back(y + x + z);
    js_.velocity.push_back(y - x + z);
    js_.velocity.push_back(y - x - z);
    js_.velocity.push_back(y + x - z);

    printf("%f\n", y+x+z);

    vel_pub_.publish(js_);
    //RRsender_.update(y + x + z);
    //self._velocityRL = y - x + z;
    //self._velocityFR = y - x - z;
    //self._velocityFL = y + x - z;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  //;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mech_teleop");

  ros::NodeHandle nh;
  InvKinematics myInvKin (nh);

  ros::Subscriber joy_sub = nh.subscribe("joy", 100, &InvKinematics::update, &myInvKin);


  ros::spin();

  return 0;
}
