#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace boost;
using namespace ros;

float randomizePosition()
{
    srand(6832*time(NULL)); // set initial seed value to 5323
    return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace acastro_ns
{
class Team
{
public:
  string team_name;
  vector<string> player_names;
  ros::NodeHandle n;

  Team(string team_name_in)
  {
    team_name = team_name_in;
    // read the team players
    n.getParam("/team_" + team_name, player_names);
  }

  bool playerBelongsToTeam(string player_name)
  {
    for (size_t i = 0; i < player_names.size(); i++)
    {
      if (player_names[i] == player_name)
      {
        return true;
      }
    }
    return false;
  }

private:
};

class Player
{
public:
  // properties
  string player_name;
   ros::NodeHandle n;

  Player(string player_name_in)
  {
    player_name = player_name_in;
  } // Constructor

  void setTeamName(string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
    {
      team_name = team_name_in;
    }
    else
    {
      cout << "Cannot set team name" << team_name_in << endl;
    }
  }

  void setTeamName(int team_index)
  {
    if (team_index == 0)
      setTeamName("red");
    else if (team_index == 1)
      setTeamName("green");
    else if (team_index == 2)
      setTeamName("blue");
    else
      setTeamName("");
  }

  string getTeamName()
  {
    return team_name;
  };

private:
  string team_name;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;

  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  boost::shared_ptr<ros::Publisher> vis_pub;


  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {
    setTeamName(team_name_in);
    vis_pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");
    (*vis_pub) = n.advertise<visualization_msgs::Marker>( "/player_names", 0 );

    if (team_red->playerBelongsToTeam(player_name))
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    }
    else if (team_green->playerBelongsToTeam(player_name))
    {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    }
    else if (team_blue->playerBelongsToTeam(player_name))
    {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    }
    else
    {
      cout << "something's wrong in team parametrizations!!!" << endl;
    }

    setTeamName(team_mine->team_name);

    //define intial position
            float sx = randomizePosition();
            float sy = randomizePosition();
            tf::Transform T1;
            T1.setOrigin( tf::Vector3(sx, sy, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, M_PI);
            T1.setRotation(q);

            //define global movement
            tf::Transform Tglobal = T1;
            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
            printInfo();
  }

  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name << endl);
    ROS_INFO_STREAM("I'm hunting " << team_preys->team_name << " and i am being chased by " << team_hunters->team_name << endl);
  }

  std::tuple<float, float> getDistanceAndAngleToPlayer(string other_name)
  {
    tf::StampedTransform T0;
try{
  listener.lookupTransform(player_name, other_name, ros::Time(0), T0);
}
catch (tf::TransformException ex){
  ROS_ERROR("%s", ex.what());
  ros::Duration(0.1).sleep();
  return {1000.0, 0.0};
}

float d= sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());
float a= atan2 (T0.getOrigin().y(), T0.getOrigin().x());
return {d,a};
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new message");
    //publicar uma transformacao


//STEP 1: Find out where I am
tf::StampedTransform T0;
try{
  listener.lookupTransform("/world", player_name, ros::Time(0), T0);
}
catch (tf::TransformException ex){
  ROS_ERROR("%s", ex.what());
  ros::Duration(0.1).sleep();
}

//STEP2: define how i want to move
// For each prey, find the closest. Then follow her
vector<float> distance_to_preys;
vector<float> angle_to_preys;
// vector<float> distance_to_hunters;
// vector<float> angle_to_hunters;

for (size_t i=0; i < team_preys->player_names.size(); i++)
{
  ROS_WARN_STREAM("team_preys = " << team_preys->player_names[i]);
  std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_preys->player_names[i]);
  distance_to_preys.push_back(std::get<0>(t));
  angle_to_preys.push_back(std::get<1>(t));
}

//  for (size_t i =0; i< team_hunters->player_names.size(); i++)
//             {
//                 ROS_WARN_STREAM("team_hunters = " << team_hunters->player_names[i]);

//                 std::tuple<float, float> tt = getDistanceAndAngleToPlayer(team_hunters->player_names[i]);
//                 distance_to_hunters.push_back( std::get<0>(tt));
//                 angle_to_hunters.push_back( std::get<1>(tt));
//             }

int idx_closest_prey = 0;
float distance_closest_prey = 1000;
for (size_t i=0; i< distance_to_preys.size(); i++)
{
  if (distance_to_preys[i] < distance_closest_prey)
  {
  idx_closest_prey = i;
  distance_closest_prey = distance_to_preys[i];
  }
}

// int idx_closest_hunter = 0;
// float distance_closest_hunter = 1000;
// for (size_t i=0; i< distance_to_hunters.size(); i++)
// {
//   if (distance_to_hunters[i] < distance_closest_hunter)
//   {
//   idx_closest_hunter = i;
//   distance_closest_hunter = distance_to_hunters[i];
//   }
// }


float angle;
// if (distance_closest_prey<distance_closest_hunter)
// {
  angle = angle_to_preys[idx_closest_prey];
// }
// else
// {
//   angle = angle_to_hunters[idx_closest_hunter]+M_PI/32;
// }

float dx=10;

   //STEP2.5: check values
    float dx_max = msg->dog;
    dx > dx_max ? dx = dx_max : dx = dx;

    double amax = M_PI/30;
    fabs(angle) > fabs(amax) ? angle = amax * angle / fabs(angle): angle = angle;

//STEP 3: define local movement
    tf::Transform T1;
    T1.setOrigin( tf::Vector3(dx, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);
  

  //STEP 4: define global movement
  tf::Transform Tglobal= T0*T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

    visualization_msgs::Marker marker;
marker.header.frame_id = player_name;
marker.header.stamp = ros::Time();
marker.ns = player_name;
marker.id = 0;
marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
marker.action = visualization_msgs::Marker::ADD;
// marker.pose.position.x = 1;
// marker.pose.position.y = 1;
// marker.pose.position.z = 1;
// marker.pose.orientation.x = 0.0;
// marker.pose.orientation.y = 0.0;
// marker.pose.orientation.z = 0.0;
// marker.pose.orientation.w = 1.0;
// marker.scale.x = 1;
// marker.scale.y = 0.1;
marker.scale.z = 0.6;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker.text=player_name;
//only if using a MESH_RESOURCE marker type:
//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
vis_pub->publish( marker );

  };

private:
};

} // namespace acastro


int main(int argc, char **argv)
{
  ros::init(argc, argv, "acastro");
  ros::NodeHandle n;
  acastro_ns::MyPlayer player("acastro", "green");

 ros::Subscriber sub = n.subscribe("/make_a_play", 100, &acastro_ns::MyPlayer::makeAPlayCallback, &player);


#if 0
  cout << "Hello World from " << player.player_name << " of team " << player.getTeamName() << endl;

   acastro::Team team_red("red");
#endif

  player.printInfo();
ros::Rate r(20);

  while (ros::ok())
  {
  ros::spinOnce();
        r.sleep();
  }

  return 1;
}