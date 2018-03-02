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
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#define DEFAULT_TIME 0.05
using namespace std;
using namespace boost;
using namespace ros;
using namespace tf;
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
  TransformBroadcaster br;
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
    sub = shared_ptr<Subscriber>(new Subscriber());
    *sub = n.subscribe("/make_a_play", 1000, &MyPlayer::move, this);
    vis_pub = n.advertise<visualization_msgs::Marker>("/bocas", 0);
    warp();

    piropo("do not fear, nando fabricio is here", 1);
  }


  double getAngleToPLayer(string other_player, double time_to_wait=DEFAULT_TIME)
      {
        StampedTransform t; //The transform object
        Time now = Time(0); //get the latest transform received

        try{
          listener.waitForTransform(name, other_player, now, Duration(time_to_wait));
          listener.lookupTransform(name, other_player, now, t);
        }
        catch (TransformException& ex){
          ROS_ERROR("%s",ex.what());
          return NAN;
        }

        return atan2(t.getOrigin().y(), t.getOrigin().x());
      }

  void warp()
  {
    transform.setOrigin(Vector3(x, y, 0.0));
    Quaternion q;
    q.setRPY(0, 0, alfa);
    transform.setRotation(q);
    br.sendTransform(StampedTransform(transform, Time::now(), "world", name));
  }

  void piropo(string boca, int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = name;
    marker.header.stamp = Time();
    marker.ns = name;
    marker.lifetime = Duration(2.0);
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.3;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = boca;
    vis_pub.publish(marker);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
  {
    double max_dist = msg->turtle;
    double max_delta_alpha = M_PI / 30;


    /* AI */
    double intended_dist_x = 3;
    double intended_dist_y = 3;
    double intended_delta_alpha = getAngleToPLayer("blourenco");
    if(isnan(intended_delta_alpha))
      intended_delta_alpha = 0;
    /******/

    /* constrains */
    double actual_dist_x = abs(intended_dist_x) > max_dist ? max_dist : intended_dist_x;
    double actual_dist_y = abs(intended_dist_y) > max_dist ? max_dist : intended_dist_y;
    double actual_delta_alpha = fabs(intended_delta_alpha) > fabs(max_delta_alpha) ?
                                    max_delta_alpha * intended_delta_alpha / fabs(intended_delta_alpha) :
                                    intended_delta_alpha;
    /*************/

    /* bounds */

    /**********/
    Transform displacement_transform;

    displacement_transform.setOrigin(Vector3(actual_dist_x, actual_dist_y, 0.0));
    Quaternion q;
    q.setRPY(0, 0, actual_delta_alpha);
    displacement_transform.setRotation(q);
    transform = transform * displacement_transform;
    br.sendTransform(StampedTransform(transform, Time::now(), "world", name));
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    piropo("NANDO FABRICIO vai te apanhar blourenco", 0);
  }

private:
  float x, y;
  string type;
  double speedx, speedy, alfa;
  NodeHandle n;
  shared_ptr<Subscriber> sub;
  Transform transform;
  Publisher vis_pub;
  TransformListener listener;
};

}  // end of namespace

int main(int argc, char** argv)
{
  string name = "amartins";
  string team = "blue";
  string type = "turtle";
  init(argc, argv, name);
  NodeHandle n;
  // Creating an instance of class Player
  rws_amartins::MyPlayer my_player(name, team, type);

  spin();
}