#include <ros/ros.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace boost;
using namespace ros;

namespace acastro
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
  }
  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name << endl);
    ROS_INFO_STREAM("I'm hunting " << team_preys->team_name << " and i am being chased by " << team_hunters->team_name << endl);
  }

private:
};

} // namespace acastro

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acastro");
  ros::NodeHandle n;
  acastro::MyPlayer player("acastro", "green");
#if 0
  cout << "Hello World from " << player.player_name << " of team " << player.getTeamName() << endl;

   acastro::Team team_red("red");
#endif

  while (ros::ok())
  {
    ros::Duration(1).sleep();
    player.printInfo();
  }

  return 1;
}