#pragma once

#include <mc_control/fsm/State.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include <Eigen/Core>

/**
 * Configuration for the shaking motion (per-joint)
 */
struct Shake
{
  double period = 1;
  double amplitude = 1.0;

  void load(const mc_rtc::Configuration & config)
  {
    period = config("period");
    amplitude = config("amplitude");
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration c;
    c.add("period", period);
    c.add("amplitude", amplitude);
    return c;
  }
};

namespace mc_rtc
{
template<>
struct ConfigurationLoader<Shake>
{
  static Shake load(const mc_rtc::Configuration & config)
  {
    Shake shake;
    shake.load(config);
    return shake;
  }

  static mc_rtc::Configuration save(const Shake & object)
  {
    return object.save();
  }
};
} // namespace mc_rtc

/**
 * Configuration for a posture
 */
struct PostureConfig
{
  double t;
  std::map<std::string, double> posture;
  std::map<std::string, Shake> shake;

  void load(const mc_rtc::Configuration & config)
  {
    t = config("time");
    posture = config("posture");
    if(config.has("shake"))
    {
      shake = config("shake");
    }
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration c;
    c.add("time", t);
    c.add("posture", posture);
    c.add("shake", shake);
    return c;
  }
};

namespace mc_rtc
{
template<>
struct ConfigurationLoader<PostureConfig>
{
  static PostureConfig load(const mc_rtc::Configuration & config)
  {
    PostureConfig shake;
    shake.load(config);
    return shake;
  }

  static mc_rtc::Configuration save(const PostureConfig & object)
  {
    return object.save();
  }
};
} // namespace mc_rtc

struct SlowDance_InterpolatePosture : mc_control::fsm::State
{

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  using PostureInterpolator = mc_trajectory::SequenceInterpolator<Eigen::VectorXd>;
  PostureInterpolator interpolator_;
  double t_{0};
  bool autoplay_ = true;
  bool updatePosture_ = true;
  bool improvise_ = false;
  bool repeat_ = false;

  std::vector<PostureConfig> postureSequence_;
};
