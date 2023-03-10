#include "LoadRobotState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/RobotLoader.h>

void LoadRobotState::start(mc_control::fsm::Controller & ctl_)

{
  if(ctl_.robot().name() == "panda_default")
  {
    ctl_.loadRobot(mc_rbdyn::RobotLoader::get_robot_module("HRP4Comanoid"), "hrp4");
  }
  else
  {
    ctl_.loadRobot(mc_rbdyn::RobotLoader::get_robot_module("PandaDefault"), "panda_default");
  }

  for(auto & robot : ctl_.robots())
  {
    if(config_.has(robot.name()))
    {
      if(config_(robot.name()).has("init_pos"))
      {
        robot.posW(config_(robot.name())("init_pos"));
      }
    }
  }
}

bool LoadRobotState::run(mc_control::fsm::Controller & ctl_)
{
  output("OK");
  return true;
}
void LoadRobotState::teardown(mc_control::fsm::Controller & ctl_)
{
}

EXPORT_SINGLE_STATE("LoadRobotState", LoadRobotState)
