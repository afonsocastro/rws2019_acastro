#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_listener.h>

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


  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {
    setTeamName(team_name_in);

    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

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
float dx=0.3;
float angle = M_PI/6;


//STEP 3: define local movement
    tf::Transform T1;
    T1.setOrigin( tf::Vector3(-dx, 2*dx, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);
  

  //STEP 4: define global movement
  tf::Transform Tglobal= T0*T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
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