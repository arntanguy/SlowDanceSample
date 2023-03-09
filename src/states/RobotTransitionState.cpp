#include "RobotTransitionState.h"
#include <mc_control/fsm/Controller.h>

void RobotTransitionState::start(mc_control::fsm::Controller & ctl_)

{
  output(ctl_.robot().name());
}

bool RobotTransitionState::run(mc_control::fsm::Controller & ctl_)
{

  return true;
}

void RobotTransitionState::teardown(mc_control::fsm::Controller & ctl_)
{
}

EXPORT_SINGLE_STATE("RobotTransitionState", RobotTransitionState)
