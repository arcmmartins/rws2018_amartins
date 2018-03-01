#include <iostream>

using namespace std;

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

  string name;  // A public atribute

private:
  string team;
};

// Class myPlayer extends class Player
class myPlayer : public Player
{
public:
  myPlayer(string name, string team) : Player(name)
  {
    setTeamName(team);
  }
};

int main()
{
  // Creating an instance of class Player
  Player player("amartins");
  player.setTeamName("red");
  player.setTeamName(2);
  cout << "player.name is " << player.name << endl;
  cout << "team is " << player.getTeamName() << endl;

  myPlayer my_player("amartins", "green");
  std::cout << "my_player.name is " << my_player.name << std::endl;
  std::cout << "team is " << my_player.getTeamName() << std::endl;
}