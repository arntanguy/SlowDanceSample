#include "SlowDance_InterpolatePosture.h"

#include <mc_rtc/io_utils.h>
#include "../SlowDance.h"

void SlowDance_InterpolatePosture::start(mc_control::fsm::Controller & ctl)
{
  // Parse the desired posture sequence.
  // We expect a vector of PostureConfig
  // Example in yaml:
  //  posture_sequence:
  //    - time: 0.0
  //      posture:
  //        R_SHOULDER_P: -0.25
  //        L_SHOULDER_P: -0.21
  //        R_ELBOW_P: -0.55
  //        L_ELBOW_P: -0.20
  //    - time: 2.0
  //      posture:
  //        L_SHOULDER_R: 1.38
  //        L_SHOULDER_Y: 1.22
  //      shake:
  //        NECK_Y:
  //          period: 0.1 # s
  //          amplitude: 1 # rad
  config_("autoplay", autoplay_);
  postureSequence_ = config_("posture_sequence");
  // Get the list of actuated joints
  const auto & rjo = ctl.robot().refJointOrder();
  // Create a vector used to store the desired value for each actuated joint
  Eigen::VectorXd desiredPosture(rjo.size());
  // Initialize with current robot posture
  for(int i = 0; i < rjo.size(); ++i)
  {
    desiredPosture(i) = ctl.robot().mbc().q[ctl.robot().jointIndexInMBC(i)][0];
  }

  // Create the interpolator values
  PostureInterpolator::TimedValueVector interpolatorValues;
  // For each timed posture in the sequence
  for(const auto & postureConfig : postureSequence_)
  {
    double t = postureConfig.t;
    const auto & postureMap = postureConfig.posture;
    // For each actuated joint
    for(int i = 0; i < rjo.size(); ++i)
    {
      const auto & actuatedJoint = rjo[i];
      // Check if we have a desired posture in the configuration
      if(postureMap.count(actuatedJoint))
      {
        // If so, put the desired joint value for this actuated joint
        desiredPosture(i) = postureMap.at(actuatedJoint);
      }
      else
      {
        // Otherwise use the current joint value
        desiredPosture(i) = ctl.robot().mbc().q[ctl.robot().jointIndexInMBC(i)][0];
      }
    }
    // Add the current posture to the interpolator values
    interpolatorValues.emplace_back(t, desiredPosture);
  }
  // Put all desired postures in the interpolator
  interpolator_.values(interpolatorValues);

  // Load the configuration for the posture task.
  // See https://jrl-umi3218.github.io/mc_rtc/json.html#MetaTask/PostureTask for
  // supported values
  // Example in yaml:
  //   posture_task:
  //     stiffness: 100
  auto & postureTask = *ctl.getPostureTask(ctl.robot().name());
  postureTask.load(ctl.solver(), config_("posture_task", mc_rtc::Configuration{}));

  ctl.gui()->addElement(
      this, {name()},
      mc_rtc::gui::Checkbox(
          "Play", [this]() { return autoplay_; }, [this]() { autoplay_ = !autoplay_; }),
      mc_rtc::gui::Checkbox(
          "Update posture", [this]() { return updatePosture_; }, [this]() { updatePosture_ = !updatePosture_; }),
      mc_rtc::gui::NumberInput(
          "Time", [this]() { return t_; }, [this](double t) { t_ = t; }),
      mc_rtc::gui::NumberSlider(
          "Time selector", [this]() { return t_; }, [this](double t) { t_ = t; }, 0,
          interpolator_.values().back().first));

  output("OK");
}

bool SlowDance_InterpolatePosture::run(mc_control::fsm::Controller & ctl_)
{
  const auto & rjo = ctl_.robot().refJointOrder();

  // Compute the interpolated posture at the current time
  auto desiredPosture = interpolator_.compute(t_);

  // Shake
  auto currPostureSeq =
      std::find_if(postureSequence_.begin(), postureSequence_.end(), [this](const auto & p) { return p.t > t_; });
  currPostureSeq--;
  if(currPostureSeq != postureSequence_.end())
  {
    const auto & shakeMap = currPostureSeq->shake;
    // mc_rtc::log::info("Should shake (t={}, posture t= {})", t_, currPostureSeq->t);
    // mc_rtc::log::info("Joints: {}", mc_rtc::io::to_string(shakeMap, [](const auto & m) { return m.first; }));
    // For each actuated joint
    for(int i = 0; i < rjo.size(); ++i)
    {
      // Shake
      const auto & actuatedJoint = rjo[i];
      if(shakeMap.count(actuatedJoint))
      {
        const auto & shakeConfig = shakeMap.at(actuatedJoint);
        // Shake value such that it starts with
        // - Shake = 0 for t_ = currPostureSeq->t (no motion initially)
        // - It shakes with period shakeConfig.period around the current joint
        // trajectory value
        // - It shakes with amplitude shakeConfig.amplitude
        double shakeVal =
            shakeConfig.amplitude * sin(2 * mc_rtc::constants::PI / shakeConfig.period * (t_ - currPostureSeq->t));
        desiredPosture(i) += shakeVal;
        // mc_rtc::log::info("Shaking joint {} : {}", actuatedJoint, shakeVal);
      }
    }
  }

  // Get the posture task
  auto & postureTask = *ctl_.getPostureTask(ctl_.robot().name());
  // Copy the current posture target
  auto posture = postureTask.posture();

  // For each actuated joint
  for(int i = 0; i < rjo.size(); ++i)
  {
    const auto & actuatedJoint = rjo[i];
    // Set the posture target for this actuated joint to its interpolated value
    posture[ctl_.robot().jointIndexInMBC(i)][0] = desiredPosture[i];
  }

  // Change the posture target in the posture task
  if(updatePosture_)
  {
    postureTask.posture(posture);
  }

  if(autoplay_)
  {
    t_ += ctl_.timeStep;
  }
  return t_ >= interpolator_.values().back().first;
}

void SlowDance_InterpolatePosture::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("SlowDance_InterpolatePosture", SlowDance_InterpolatePosture)
