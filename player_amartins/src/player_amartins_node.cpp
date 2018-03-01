#include <iostream>
#include <sstream>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// ros
#include <rws2018_libs/team.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

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
      cout << "wrong team index given. Cannot set team" << endl;
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
      cout << "cannot set team name to " << team << endl;
      return 0;
    }
  }

  // Gets team name (accessor)
  string getTeamName()
  {
    return team;
  }

  string name; // A public atribute

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
  MyPlayer(string name, string team) : Player(name)
  {
    setTeamName(team);
    red_team = shared_ptr<Team>(new Team("red"));
    blue_team = shared_ptr<Team>(new Team("blue"));
    green_team = shared_ptr<Team>(new Team("green"));
  }

  void move(void)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(3, 5, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
  }
};

} // end of namespace

int main(int argc, char **argv)
{
  string name = "amartins";
  string team = "blue";
  ros::init(argc, argv, name);
  ros::NodeHandle n;
  // Creating an instance of class Player
  rws_amartins::MyPlayer my_player(name, team);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    my_player.move();
    ros::spinOnce();
    loop_rate.sleep();
  }
}