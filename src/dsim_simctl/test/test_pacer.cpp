// The pacer decides how fast the simulated world runs. These tests drive it
// with a fake world that runs to whatever instant it is given, so the
// arithmetic can be checked without Gazebo.
//
// What would make them fail: a target that advances at the wrong rate; a
// target that can run ahead of the world without bound, so an interruption is
// repaid as a sprint; a target that is dragged forward by a world that
// overran, which makes the overrun free and every speed too fast; a resync
// that does not drop the accumulated lead; and a target that changes when no
// wall time has passed, which would make an extra tick worth extra speed.

#include <cmath>

#include <gtest/gtest.h>
#include <dsim_simctl/pacer.hpp>

using dsim_simctl::Pacer;
using dsim_simctl::PacerConfig;

namespace
{

/// A world that runs to whatever instant it is told, immediately.
struct FakeWorld
{
  double sim {0.0};
  void runTo(double t) {if (t > sim) {sim = t;}}
};

/// Run `ticks` ticks of `tick_s` wall time each; report simulated time elapsed.
double runFor(Pacer & p, FakeWorld & w, int ticks, double tick_s)
{
  const double start = w.sim;
  for (int i = 0; i < ticks; ++i) {
    w.runTo(p.target(tick_s, w.sim));
  }
  return w.sim - start;
}

}  // namespace

TEST(Pacer, RealTimeIsRealTime)
{
  // The identity case. If this is wrong nothing else can be right.
  Pacer p;
  FakeWorld w;
  p.setSpeed(1.0);
  p.resync(w.sim);
  EXPECT_NEAR(runFor(p, w, 200, 0.05), 10.0, 0.001);   // within one step
}

TEST(Pacer, EverySpeedIsAchievedToWithinOneStep)
{
  // 0.05x at a 50 ms tick is two and a half steps' worth per tick. Anything
  // that drops the half quantises the speed downwards by 20% -- which would
  // look like "slow motion is a bit slower than the slider says" and never be
  // chased.
  for (double speed : {0.05, 0.1, 0.25, 0.5, 1.0, 1.7, 2.0, 4.0}) {
    Pacer p;
    FakeWorld w;
    p.setSpeed(speed);
    p.resync(w.sim);
    const double wall = 20.0;
    EXPECT_NEAR(runFor(p, w, 400, wall / 400.0), speed * wall, 0.002)
      << "at speed " << speed;
  }
}

TEST(Pacer, NoWallTimeMeansNoExtraSpeed)
{
  // The target is a function of elapsed WALL time, not of how often it is
  // asked for. Otherwise an extra tick -- a timer that fires twice, a second
  // caller -- buys extra simulated time, and the speed depends on the tick
  // rate rather than on the slider.
  Pacer p;
  p.setSpeed(0.25);
  p.resync(10.0);
  const double first = p.target(0.05, 10.0);
  EXPECT_DOUBLE_EQ(p.target(0.0, 10.0), first);
  EXPECT_DOUBLE_EQ(p.target(0.0, 10.0), first);
}

TEST(Pacer, AWorldFarAheadIsWaitedFor)
{
  // A world well ahead of the target -- stepped from the Gazebo GUI, say --
  // must leave the target BEHIND it, because that is how the node is told to
  // keep the world stopped. It must not be dragged up to meet the world, which
  // would hand back the extra time for free.
  //
  // Bounded, though: the wait is at most max_slip_s of simulated time, not
  // however far ahead the world happens to have got.
  PacerConfig cfg;
  cfg.max_slip_s = 0.5;
  Pacer p(cfg);
  p.setSpeed(1.0);
  p.resync(0.0);
  EXPECT_LT(p.target(0.01, 100.0), 100.0);            // stay stopped
  EXPECT_GE(p.target(0.01, 100.0), 100.0 - 0.5);      // but not forever
}

TEST(Pacer, AWorldThatOverrunsIsNotRewarded)
{
  // The situation the real simulator is always in: a pause takes effect a
  // moment after it is asked for, so the world stops a little PAST the target
  // every time. That overrun is time it owes back, and it pays it by staying
  // stopped a little longer.
  //
  // Forgiving it instead -- dragging the target up to wherever the world got
  // to -- makes the overrun free, and every speed comes out high. That was a
  // real bug: 0.25x ran at 0.63x, and because the target then followed the
  // world, the published backlog looked perfectly healthy the whole time.
  Pacer p;
  p.setSpeed(0.25);
  p.resync(0.0);
  double sim = 0.0;
  const double tick = 0.05, overrun = 0.02;
  for (int i = 0; i < 400; ++i) {
    const double t = p.target(tick, sim);
    if (t > sim) {sim = t + overrun;}          // stops late, every time
  }
  EXPECT_NEAR(sim, 0.25 * 400 * tick, 0.1);    // 5 s of sim in 20 s of wall
}

TEST(Pacer, TheTargetCannotRunAwayFromTheWorld)
{
  PacerConfig cfg;
  cfg.max_slip_s = 0.5;
  Pacer p(cfg);
  p.setSpeed(4.0);
  p.resync(0.0);
  // Twenty seconds of wall time in which the world did not move at all.
  // Unclamped the target would be 80 s ahead, and the world would then run
  // flat out for a minute and a half to reach it.
  const double t = p.target(20.0, 0.0);
  EXPECT_LE(t, 0.5 + 1e-9);
  EXPECT_NEAR(p.backlog(), 0.5, 1e-9);
}

TEST(Pacer, SlipToleranceExceedsTheStatisticsInterval)
{
  // The world's position is only known as often as it publishes statistics,
  // every 0.2 s. A slip tolerance below that would clamp the target to behind
  // where the world had already got to, and the world would stall instead of
  // running slowly.
  EXPECT_GT(PacerConfig{}.max_slip_s, 0.2);
}

TEST(Pacer, ResyncDropsTheLead)
{
  Pacer p;
  p.setSpeed(1.0);
  p.resync(0.0);
  p.target(5.0, 0.0);                  // build a lead
  ASSERT_GT(p.backlog(), 0.0);
  p.resync(0.0);
  EXPECT_EQ(p.backlog(), 0.0);
  EXPECT_NEAR(p.target(0.05, 0.0), 0.05, 1e-9);
}

TEST(Pacer, ResyncAfterAResetDoesNotSprint)
{
  // The world was at 200 s and is now at 0. Without a resync the target still
  // points 200 s into the future, and `run_to_sim_time` would obediently run
  // the whole of a run that no longer exists, as fast as the machine allows.
  Pacer p;
  p.setSpeed(1.0);
  p.resync(200.0);
  p.target(0.05, 200.0);
  p.resync(0.0);
  EXPECT_NEAR(p.target(0.05, 0.0), 0.05, 1e-9);
}

TEST(Pacer, SpeedChangeDoesNotCauseAJump)
{
  // The target is absolute simulated time, so a new speed changes only the
  // rate it grows at -- never its value.
  Pacer p;
  FakeWorld w;
  p.setSpeed(1.0);
  p.resync(0.0);
  runFor(p, w, 100, 0.05);
  const double before = w.sim;
  p.setSpeed(0.1);
  EXPECT_LE(p.target(0.05, w.sim) - before, 0.006);   // 0.1 * 0.05 s, plus a step
}

TEST(Pacer, TargetsLandOnTheStepGrid)
{
  // The world advances in whole physics steps. A target between two of them
  // makes the world stop a fraction past where it was asked to, and simulated
  // time drifts off the millisecond grid every timestamp is written on.
  PacerConfig cfg;
  cfg.step_s = 0.001;
  Pacer p(cfg);
  p.setSpeed(0.333);
  p.resync(0.0);
  for (int i = 0; i < 50; ++i) {
    const double t = p.target(0.05, 0.0);
    const double grid = std::round(t / cfg.step_s) * cfg.step_s;
    ASSERT_NEAR(t, grid, 1e-12) << "target " << t << " is not a whole step";
  }
}

TEST(Pacer, RejectsNonsenseSpeeds)
{
  Pacer p;
  p.setSpeed(2.0);
  p.setSpeed(0.0);
  EXPECT_DOUBLE_EQ(p.speed(), 2.0);   // "never advance" is what pause is for
  p.setSpeed(-1.0);
  EXPECT_DOUBLE_EQ(p.speed(), 2.0);
  p.setSpeed(std::nan(""));
  EXPECT_DOUBLE_EQ(p.speed(), 2.0);
}

TEST(Pacer, FirstTickAdoptsTheWorldsTime)
{
  // No resync() called: the pacer must not treat the world's current time as
  // a 96-second debt just because its own target still reads zero.
  Pacer p;
  p.setSpeed(1.0);
  EXPECT_NEAR(p.target(0.05, 96.132), 96.182, 1e-9);
}
