// The referee's geometry. Every expected number below is derived by hand in
// the comment next to it, not read back from the code under test.
#include <gtest/gtest.h>
#include <cmath>
#include "dsim_eval/obstacle.hpp"

using dsim_eval::ClearanceTracker;
using dsim_eval::Obstacle;

namespace
{
Obstacle sphere(const std::string & name, double x, double y, double z, double r,
  double vx = 0, double vy = 0, double vz = 0)
{
  Obstacle o;
  o.name = name; o.type = "sphere";
  o.x = x; o.y = y; o.z = z; o.sx = o.sy = o.sz = r;
  o.vx = vx; o.vy = vy; o.vz = vz;
  return o;
}
}  // namespace

TEST(Obstacle, ASphereReportsSignedDistance)
{
  const auto o = sphere("s", 0, 0, 0, 1.0);
  // 2 m from the centre of a unit sphere: 1 m outside.
  EXPECT_NEAR(o.signedDistance(2, 0, 0, 0.0), 1.0, 1e-12);
  // 0.3 m from the centre: 0.7 m INSIDE, and it must say so with a sign.
  EXPECT_NEAR(o.signedDistance(0.3, 0, 0, 0.0), -0.7, 1e-12);
  EXPECT_NEAR(o.signedDistance(0, 0, 0, 0.0), -1.0, 1e-12);
}

TEST(Obstacle, AMovingSphereIsWhereItsVelocitySaysAtThatTime)
{
  // Centre starts at x=4 and moves +0.25 m/s: at t=8 it is at x=6.
  const auto o = sphere("F", 4, 5, 0, 0.9, 0.25, 0, 0);
  double cx, cy, cz;
  o.centreAt(8.0, cx, cy, cz);
  EXPECT_NEAR(cx, 6.0, 1e-12);
  EXPECT_NEAR(cy, 5.0, 1e-12);
  EXPECT_NEAR(cz, 0.0, 1e-12);
  // The point (4, 5, 0) is the centre at t=0 (distance -0.9) and 2 m from it
  // at t=8 (distance 1.1). An obstacle that ignores its velocity gets the
  // second one wrong by exactly 2 m.
  EXPECT_NEAR(o.signedDistance(4, 5, 0, 0.0), -0.9, 1e-12);
  EXPECT_NEAR(o.signedDistance(4, 5, 0, 8.0), 1.1, 1e-12);
  // Negative time runs the motion backwards: the Gazebo body was placed at
  // pos0 - vel*start_s and has been moving since sim t=0.
  o.centreAt(-10.0, cx, cy, cz);
  EXPECT_NEAR(cx, 1.5, 1e-12);
}

TEST(Obstacle, StaticShapesIgnoreTime)
{
  const auto o = sphere("s", 1, 2, 3, 0.5);
  EXPECT_FALSE(o.moving());
  EXPECT_NEAR(o.signedDistance(3, 2, 3, 0.0), 1.5, 1e-12);
  EXPECT_NEAR(o.signedDistance(3, 2, 3, 123.0), 1.5, 1e-12);
}

TEST(Obstacle, ABoxIsSignedToo)
{
  Obstacle b;
  b.type = "box"; b.sx = 2.0; b.sy = 4.0; b.sz = 6.0;   // half extents 1, 2, 3
  EXPECT_NEAR(b.signedDistance(1.5, 0, 0, 0), 0.5, 1e-12);       // 0.5 past the x face
  EXPECT_NEAR(b.signedDistance(1.5, 2.5, 0, 0), std::hypot(0.5, 0.5), 1e-12);  // edge
  // Inside: nearest face is x (1 - 0.8 = 0.2), not y (2 - 0) or z (3 - 0).
  EXPECT_NEAR(b.signedDistance(0.8, 0, 0, 0), -0.2, 1e-12);
  EXPECT_NEAR(b.signedDistance(0, 0, 0, 0), -1.0, 1e-12);        // centre: 1 to the x faces
}

TEST(Obstacle, ACylinderIsSignedToo)
{
  Obstacle c;
  c.type = "cylinder"; c.sx = 0.3; c.sz = 3.0;   // radius 0.3, half height 1.5
  EXPECT_NEAR(c.signedDistance(0.5, 0, 0, 0), 0.2, 1e-12);       // radially outside
  EXPECT_NEAR(c.signedDistance(0, 0, 2.0, 0), 0.5, 1e-12);       // above the top
  EXPECT_NEAR(c.signedDistance(0.1, 0, 0, 0), -0.2, 1e-12);      // inside, wall is nearer
  EXPECT_NEAR(c.signedDistance(0, 0, 1.4, 0), -0.1, 1e-12);      // inside, cap is nearer
}

TEST(ClearanceTracker, SubtractsTheVehicleRadius)
{
  // Unit sphere, 0.3 m vehicle. A point 1.25 m out has 0.25 m to the surface
  // and the envelope is 0.3 m wide: 5 cm of penetration.
  ClearanceTracker tr({sphere("s", 0, 0, 0, 1.0)}, 0.3);
  const auto s = tr.update(1.25, 0, 0, 0.0);
  EXPECT_NEAR(s.clearance, -0.05, 1e-12);
  EXPECT_EQ(s.nearest, "s");
  ASSERT_EQ(tr.hits().size(), 1u);
}

TEST(ClearanceTracker, NamesTheNearestNotTheFirst)
{
  ClearanceTracker tr({sphere("far", 10, 0, 0, 1.0), sphere("near", 0, 3, 0, 1.0)}, 0.0);
  EXPECT_EQ(tr.update(0, 0, 0, 0.0).nearest, "near");
  EXPECT_NEAR(tr.update(0, 0, 0, 0.0).clearance, 2.0, 1e-12);
}

TEST(ClearanceTracker, OneExcursionIsOneHit)
{
  ClearanceTracker tr({sphere("s", 0, 0, 0, 1.0)}, 0.0);
  // Approach along +x from 2.0 to 0.2 (inside from x < 1.0), then back out.
  for (int i = 0; i <= 180; ++i) {tr.update(2.0 - 0.01 * i, 0, 0, 0.1 * i);}
  ASSERT_EQ(tr.hits().size(), 1u) << "500 samples inside one sphere is one collision";
  const auto & h = tr.hits()[0];
  EXPECT_EQ(h.with, "s");
  // Entry is the first sample with x < 1.0, i.e. x = 0.99 at i = 101.
  EXPECT_NEAR(h.x, 0.99, 1e-9);
  EXPECT_NEAR(h.t, 10.1, 1e-9);
  // Deepest point reached is x = 0.2: 0.8 m in.
  EXPECT_NEAR(h.depth, 0.8, 1e-9);
  for (int i = 0; i <= 180; ++i) {tr.update(0.2 + 0.01 * i, 0, 0, 18.0 + 0.1 * i);}
  EXPECT_EQ(tr.hits().size(), 1u) << "leaving is not a second hit";
  // Re-entering is.
  tr.update(0.5, 0, 0, 40.0);
  EXPECT_EQ(tr.hits().size(), 2u);
  EXPECT_NEAR(tr.minClearance(), -0.8, 1e-9);
}

TEST(ClearanceTracker, ResetForgetsEverything)
{
  ClearanceTracker tr({sphere("s", 0, 0, 0, 1.0)}, 0.0);
  tr.update(0, 0, 0, 0.0);
  ASSERT_EQ(tr.hits().size(), 1u);
  tr.reset();
  EXPECT_TRUE(tr.hits().empty());
  EXPECT_TRUE(std::isinf(tr.minClearance()));
  // After a reset the same position is a NEW entry, not a continuation.
  tr.update(0, 0, 0, 0.0);
  EXPECT_EQ(tr.hits().size(), 1u);
}

TEST(ClearanceTracker, NoObstaclesMeansInfiniteClearanceAndNoHits)
{
  ClearanceTracker tr({}, 0.3);
  const auto s = tr.update(0, 0, 0, 0.0);
  EXPECT_TRUE(std::isinf(s.clearance));
  EXPECT_TRUE(s.nearest.empty());
  EXPECT_TRUE(tr.hits().empty());
}

// The seed plan through the fence3d scenario, worked by hand.
//
// The straight seed flies x = 0.5 + 0.9 t, y = 5, z = 0.5. Fence sphere F5
// sits at (4, 5, 0) with r = 0.9 moving +0.25 m/s in x. The vehicle is 0.3 m.
//   horizontal gap  g(t) = 0.5 + 0.9 t - (4 + 0.25 t) = 0.65 t - 3.5
//   distance^2      = g^2 + 0.5^2
//   collision when  sqrt(g^2 + 0.25) < 0.9 + 0.3 = 1.2  <=>  |g| < sqrt(1.19)
//   =>  t in ((3.5 - 1.09087)/0.65, (3.5 + 1.09087)/0.65) = (3.7064, 7.0629)
// Deepest at g = 0, t = 3.5/0.65 = 5.3846: clearance 0.5 - 1.2 = -0.7.
TEST(ClearanceTracker, TheSeedPlanHitsTheFenceWhenTheArithmeticSaysItDoes)
{
  std::vector<Obstacle> fence;
  for (int i = 0; i < 11; ++i) {
    fence.push_back(sphere("F" + std::to_string(i), 4.0, 0.5 + 0.9 * i, 0.0, 0.9, 0.25, 0, 0));
  }
  ClearanceTracker tr(fence, 0.3);
  double first_hit_t = -1.0;
  for (int i = 0; i <= 1000; ++i) {
    const double t = 0.01 * i;
    const auto s = tr.update(0.5 + 0.9 * t, 5.0, 0.5, t);
    if (s.clearance < 0.0 && first_hit_t < 0.0) {first_hit_t = t;}
  }
  ASSERT_FALSE(tr.hits().empty()) << "the straight seed goes through the fence";
  EXPECT_EQ(tr.hits()[0].with, "F5");
  EXPECT_NEAR(first_hit_t, 3.71, 0.011);            // first 10 ms sample past 3.7064
  EXPECT_NEAR(tr.minClearance(), -0.7, 1e-3);
  EXPECT_NEAR(tr.hits()[0].depth, 0.7, 1e-3);
  // F4 and F6 are 0.9 m to either side in y; at closest approach the gap to
  // them is sqrt(0.9^2 + 0.5^2) - 1.2 = -0.171: they are hit too, F5 first.
  EXPECT_GE(tr.hits().size(), 3u);
  // And a vehicle that had been told the fence is at x = 4 for all time
  // (velocity ignored) would first hit at 0.5 + 0.9 t = 4 - 1.09087, t = 2.68.
  // The 3.71 above is only right because the fence moved.
}
