#include <iostream>
#include <sstream>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// ros
#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>
#include <rws2018_msgs/GameQuery.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
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
  boost::shared_ptr<Team> my_team;
  boost::shared_ptr<Team> my_preys;
  boost::shared_ptr<Team> my_hunters;
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
    alfa = M_PI;
    ROS_INFO_STREAM("( " << name << " , " << team << " , " << type << " , x: " << x << " , y: " << y << " )");

    red_team = shared_ptr<Team>(new Team("red"));
    blue_team = shared_ptr<Team>(new Team("blue"));
    green_team = shared_ptr<Team>(new Team("green"));

    if (red_team->playerBelongsToTeam(name))
    {
      my_team = red_team;
      my_preys = green_team;
      my_hunters = blue_team;
      setTeamName("red");
    }
    else if (green_team->playerBelongsToTeam(name))
    {
      my_team = green_team;
      my_preys = blue_team;
      my_hunters = red_team;
      setTeamName("green");
    }
    else if (blue_team->playerBelongsToTeam(name))
    {
      my_team = blue_team;
      my_preys = red_team;
      my_hunters = green_team;
      setTeamName("blue");
    }

    sub = shared_ptr<Subscriber>(new Subscriber());
    *sub = n.subscribe("/make_a_play", 1000, &MyPlayer::move, this);
    vis_pub = n.advertise<visualization_msgs::Marker>("/bocas", 0);
    warp();

    piropo("do not fear, nando fabricio is here", 1);
    game_query_service = n.advertiseService("/" + name + "/game_query", &MyPlayer::respondToGameQuery, this);
  }

  bool respondToGameQuery(rws2018_msgs::GameQuery::Request &req,
                          rws2018_msgs::GameQuery::Response &res)
  {
    res.resposta = "banana";
    return true;
  }

  double getAngleToPLayer(string other_player, double time_to_wait = DEFAULT_TIME)
  {
    StampedTransform t; // The transform object
    Time now = Time(0); // get the latest transform received

    try
    {
      listener.waitForTransform(name, other_player, now, Duration(time_to_wait));
      listener.lookupTransform(name, other_player, now, t);
    }
    catch (TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return NAN;
    }

    return atan2(t.getOrigin().y(), t.getOrigin().x());
  }

  string findClosestPlayer()
  {
    double min_dist = 100000;
    string tmp_player = "moliveira";
    for (int i = 0; i < alive_preys.size(); i++)
    {
      string player_name = alive_preys[i];
      double dist = getDistanceToPLayer(player_name);
      if (isnan(dist))
        continue;
      if (dist < min_dist)
      {
        min_dist = dist;
        tmp_player = player_name;
      }
    }
    return tmp_player;
  }

  string findClosestHunter()
  {
    double min_dist = 100000;
    string tmp_player = "tmarques";
    for (int i = 0; i < alive_hunters.size(); i++)
    {
      string player_name = alive_hunters[i];
      double dist = getDistanceToPLayer(player_name);
      if (isnan(dist))
        continue;
      if (dist < min_dist)
      {
        min_dist = dist;
        tmp_player = player_name;
      }
    }
    return tmp_player;
  }

  double getDistanceToPLayer(string other_player, double time_to_wait = DEFAULT_TIME)
  {
    StampedTransform t; // The transform object
    Time now = Time(0); // get the latest transform received

    try
    {
      listener.waitForTransform(name, other_player, now, Duration(time_to_wait));
      listener.lookupTransform(name, other_player, now, t);
    }
    catch (TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return NAN;
    }

    return sqrt(t.getOrigin().y() * t.getOrigin().y() + t.getOrigin().x() * t.getOrigin().x());
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
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.3;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.text = boca;
    vis_pub.publish(marker);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr &msg)
  {
    double max_dist = msg->turtle;
    double max_delta_alpha = M_PI / 30;
    alive_preys = msg->red_alive;
    alive_hunters = msg->green_alive;
    /* AI */
    string target = findClosestPlayer();
    string closest_hunter = findClosestHunter();
    double intended_dist = 3;
    double intended_delta_alpha = getAngleToPLayer(target);
    if (isnan(intended_delta_alpha))
      intended_delta_alpha = 0;
    /******/
    if (getDistanceToPLayer(target) < getDistanceToPLayer(closest_hunter))
    {
      intended_dist = 3;
      intended_delta_alpha = getAngleToPLayer(target);
      if (isnan(intended_delta_alpha))
        intended_delta_alpha = 0;
    }
    else if (getDistanceToPLayer(closest_hunter) < 1 && getDistanceToPLayer(closest_hunter) < getDistanceToPLayer(target))
    {
      intended_dist = 3;
      intended_delta_alpha = -getAngleToPLayer(closest_hunter);
      if (isnan(intended_delta_alpha))
        intended_delta_alpha = 0;
    }

    /* constrains */
    double actual_dist = abs(intended_dist) > max_dist ? max_dist : intended_dist;
    double actual_delta_alpha = fabs(intended_delta_alpha) > fabs(max_delta_alpha) ? max_delta_alpha * intended_delta_alpha / fabs(intended_delta_alpha) : intended_delta_alpha;
    /*************/

    /* bounds */

    /**********/
    Transform displacement_transform;

    displacement_transform.setOrigin(Vector3(actual_dist, 0, 0.0));
    Quaternion q;
    q.setRPY(0, 0, actual_delta_alpha);
    displacement_transform.setRotation(q);
    transform = transform * displacement_transform;
    br.sendTransform(StampedTransform(transform, Time::now(), "world", name));
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    piropo("NANDO FABRICIO vai te apanhar " + target, 0);
  }

private:
  float x, y;
  string type;
  ros::ServiceServer game_query_service;
  vector<string> alive_preys;
  vector<string> alive_hunters;
  double alfa;
  NodeHandle n;
  shared_ptr<Subscriber> sub;
  Transform transform;
  Publisher vis_pub;
  TransformListener listener;
};

} // end of namespace

int main(int argc, char **argv)
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