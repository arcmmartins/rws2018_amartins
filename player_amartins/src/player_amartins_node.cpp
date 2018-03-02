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
    struct timeval t1;
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec);
    x = ((double)rand() / (double)RAND_MAX * 10 - 5);
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec);
    y = ((double)rand() / (double)RAND_MAX * 10 - 5);
    speedx = 0.5;
    speedy = 0.5;
    alfa = M_PI;
    ROS_INFO_STREAM("( " << name << " , " << team << " , " << type << " , x: " << x << " , y: " << y << " )");

    red_team = shared_ptr<Team>(new Team("red"));
    blue_team = shared_ptr<Team>(new Team("blue"));
    green_team = shared_ptr<Team>(new Team("green"));
    sub = shared_ptr<ros::Subscriber>(new ros::Subscriber());
    *sub = n.subscribe("/make_a_play", 1000, &MyPlayer::move, this);
    warp();
  }

  void warp()
  {
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, alfa);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
  {
    double max_dist = msg->turtle;
    double max_delta_alpha = M_PI / 30;
    /* AI */
    double intended_dist_x = 6;
    double intended_dist_y = 6;
    double intended_delta_alpha = M_PI/2;
    /******/

    /* constrains */
    double actual_dist_x = intended_dist_x > max_dist ? max_dist : intended_dist_x;
    double actual_dist_y = intended_dist_y > max_dist ? max_dist : intended_dist_y;
    double actual_delta_alpha = fabs(intended_delta_alpha) > fabs(max_delta_alpha) ?
                                    max_delta_alpha * intended_delta_alpha / fabs(intended_delta_alpha) :
                                    intended_delta_alpha;
    /*************/

    /* bounds */
    /**********/

    tf::Transform displacement_transform;

    displacement_transform.setOrigin(tf::Vector3(actual_dist_x, actual_dist_y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, actual_delta_alpha);
    displacement_transform.setRotation(q);
    transform = transform * displacement_transform;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
  }

private:
  float x, y;
  string type;
  double speedx, speedy, alfa;
  ros::NodeHandle n;
  shared_ptr<ros::Subscriber> sub;
  tf::Transform transform;
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

  ros::spin();
}