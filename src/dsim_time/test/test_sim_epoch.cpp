// What would make these fail: a detector that fires on the first sample, that
// fires on ordinary forward time, that fires repeatedly for one reset, that
// treats sample jitter as a restart, or that only recognises a reset landing
// exactly on zero.

#include <gtest/gtest.h>
#include <dsim_time/sim_epoch.hpp>

using dsim_time::SimEpoch;

TEST(SimEpoch, FirstSampleIsNotARestart)
{
  SimEpoch e;
  // There is nothing accumulated yet to throw away. A node that logged "run
  // restarted" here would do so on every launch.
  EXPECT_FALSE(e.restarted(0.0));
  EXPECT_FALSE(e.restarted(1234.5));
  EXPECT_EQ(e.epochs(), 0u);
}

TEST(SimEpoch, MonotonicTimeNeverRestarts)
{
  SimEpoch e;
  for (int i = 0; i < 10000; ++i) {
    ASSERT_FALSE(e.restarted(i * 0.004)) << "at i=" << i;
  }
  EXPECT_EQ(e.epochs(), 0u);
}

TEST(SimEpoch, BackwardsJumpFiresExactlyOnce)
{
  SimEpoch e;
  e.restarted(200.0);
  EXPECT_TRUE(e.restarted(0.02));      // the reset
  EXPECT_FALSE(e.restarted(0.03));     // the run that follows it
  EXPECT_FALSE(e.restarted(0.04));
  EXPECT_EQ(e.epochs(), 1u);
}

TEST(SimEpoch, ResetNeedNotLandOnZero)
{
  // A world reset takes effect between physics steps, and the first sample a
  // node takes afterwards is already some way into the new run. Testing for
  // `now == 0` would miss every real reset.
  SimEpoch e;
  e.restarted(96.132);
  EXPECT_TRUE(e.restarted(0.96));
}

TEST(SimEpoch, SmallBackwardsStepsAreJitterNotRestarts)
{
  SimEpoch e(0.05);
  e.restarted(10.0);
  EXPECT_FALSE(e.restarted(10.0 - 0.049));
  EXPECT_FALSE(e.restarted(10.0 - 0.030));
  EXPECT_EQ(e.epochs(), 0u);
}

TEST(SimEpoch, TheThresholdSeparatesJitterFromAReset)
{
  // Deliberately NOT a test of the exact boundary. 10.0 - 9.95 evaluates to
  // 0.050000000000000711 in binary floating point, so "exactly the threshold"
  // is not a state the caller can reach or the callee can promise. What the
  // threshold has to do is separate two populations that are three orders of
  // magnitude apart -- clock jitter of milliseconds from a reset of seconds --
  // and that is what is asserted.
  SimEpoch e(0.05);
  e.restarted(10.0);
  EXPECT_FALSE(e.restarted(9.99));     // 10 ms back: jitter
  SimEpoch f(0.05);
  f.restarted(10.0);
  EXPECT_TRUE(f.restarted(9.90));      // 100 ms back: a reset
}

TEST(SimEpoch, CountsRepeatedRestarts)
{
  SimEpoch e;
  e.restarted(50.0);
  EXPECT_TRUE(e.restarted(0.1));
  e.restarted(40.0);
  EXPECT_TRUE(e.restarted(0.1));
  e.restarted(30.0);
  EXPECT_TRUE(e.restarted(0.1));
  EXPECT_EQ(e.epochs(), 3u);
}

TEST(SimEpoch, ForgetSuppressesTheNextComparison)
{
  SimEpoch e;
  e.restarted(500.0);
  e.forget();
  // Without forget() this 500 -> 0 step is a restart. With it, there is simply
  // no previous sample to compare against.
  EXPECT_FALSE(e.restarted(0.0));
  EXPECT_EQ(e.epochs(), 0u);
  // ...and it starts comparing again from there.
  EXPECT_FALSE(e.restarted(0.1));
  EXPECT_TRUE(e.restarted(0.0));
}
