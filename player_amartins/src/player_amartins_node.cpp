#include <iostream>
#include <sstream>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// ros
#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

using namespace std;
using namespace boost;
using namespace ros;

namespace rws_amartins
{
class Player
{
public:
  // Constructor with the same name as the class
  Player(string name)
  {
    this->name = name;
  }

  int setTeamName(int team_index = 0 /*default value*/)
  {
    switch (team_index)
    {
      case 0:
        return setTeamName("red");
        break;
      case 1:
        return setTeamName("green");
        break;
      case 2:
        return setTeamName("blue");
        break;
      default:
        ROS_ERROR_STREAM("wrong team index given. Cannot set team");
        break;
    }
  }

  // Set team name, if given a correct team name (accessor)
  int setTeamName(string team)
  {
    if (team == "red" || team == "green" || team == "blue")
    {
      this->team = team;
      return 1;
    }
    else
    {
      ROS_ERROR_STREAM("cannot set team name to " << team);
      return 0;
    }
  }

  // Gets team name (accessor)
  string getTeamName()
  {
    return team;
  }

  string name;  // A public atribute

private:
  string team;
};

// Class myPlayer extends class Player
class MyPlayer : public Player
{
public:
  shared_ptr<Team> red_team;
  shared_ptr<Team> blue_team;
  shared_ptr<Team> green_team;
  tf::TransformBroadcaster br;
  MyPlayer(string name, string team, string type) : Player(name)
  {
    setTeamName(team);
    this->type = type;
    x = randomizePosition();
    y = randomizePosition();
    speedx= 0.5;
    speedy= 0.5;
    red_team = shared_ptr<Team>(new Team("red"));
    blue_team = shared_ptr<Team>(new Team("blue"));
    green_team = shared_ptr<Team>(new Team("green"));
    sub = shared_ptr<ros::Subscriber>(new ros::Subscriber());
    *sub = n.subscribe("/make_a_play", 1000, &MyPlayer::move, this);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
  {
    tf::Transform transform;
    
    if (x > 5)
    {
      speedx = -0.3;
    }
    if (x < -5)
    {
      speedx = 0.3;
    }
    if (y > 5)
    {
      speedy = -0.3;
    }
    if (y < -5)
    {
      speedy = 0.3;
    }
    transform.setOrigin(tf::Vector3(x += speedx, y +=speedy, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
  }

private:
  float x, y;
  string type;
  double speedx, speedy;
  ros::NodeHandle n;
  shared_ptr<ros::Subscriber> sub;
};

}  // end of namespace

int main(int argc, char** argv)
{
  string name = "amartins";
  string team = "blue";
  string type = "turtle";
  ros::init(argc, argv, name);
  ros::NodeHandle n;
  // Creating an instance of class Player
  rws_amartins::MyPlayer my_player(name, team, type);
  ROS_INFO_STREAM("( " << name << " , " << team << " , " << type << " )");

  ros::spin();
}