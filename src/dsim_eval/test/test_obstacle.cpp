// The referee's geometry. Every expected number below is derived by hand in
// the comment next to it, not read back from the code under test.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "dsim_eval/obstacle.hpp"

using dsim_eval::ClearanceTracker;
using dsim_eval::ControlPoint;
using dsim_eval::Obstacle;

namespace
{
/// Constant velocity is the degree-1 case: two control points.
Obstacle moving(
  const std::string & name, double x, double y, double z, double r,
  double vx = 0, double vy = 0, double vz = 0, double t0 = 0.0, double t1 = 10.0,
  const std::string & type = "sphere")
{
  Obstacle o;
  o.name = name; o.type = type; o.radius = r;
  o.control_points = {
    ControlPoint{x + vx * t0, y + vy * t0, z + vz * t0, t0},
    ControlPoint{x + vx * t1, y + vy * t1, z + vz * t1, t1}};
  return o;
}
constexpr double INF = std::numeric_limits<double>::infinity();
}  // namespace

TEST(Obstacle, ASphereReportsSignedDistance)
{
  const auto o = moving("s", 0, 0, 0, 1.0);
  // 2 m from the centre of a unit sphere: 1 m outside.
  EXPECT_NEAR(o.signedDistance(2, 0, 0, 0.0), 1.0, 1e-12);
  // 0.3 m from the centre: 0.7 m INSIDE, and it must say so with a sign.
  EXPECT_NEAR(o.signedDistance(0.3, 0, 0, 0.0), -0.7, 1e-12);
  EXPECT_NEAR(o.signedDistance(0, 0, 0, 0.0), -1.0, 1e-12);
}

TEST(Obstacle, AMovingSphereIsWhereItsControlPointsSayAtThatTime)
{
  // Centre starts at x=4 and moves +0.25 m/s: at t=8 it is at x=6.
  const auto o = moving("F", 4, 5, 0, 0.9, 0.25, 0, 0);
  const auto c = o.centreAt(8.0);
  EXPECT_NEAR(c[0], 6.0, 1e-12);
  EXPECT_NEAR(c[1], 5.0, 1e-12);
  EXPECT_NEAR(c[2], 0.0, 1e-12);
  // The point (4, 5, 0) is the centre at t=0 (distance -0.9) and 2 m from it
  // at t=8 (distance 1.1). An obstacle that ignored its motion would get the
  // second one wrong by exactly 2 m.
  EXPECT_NEAR(o.signedDistance(4, 5, 0, 0.0), -0.9, 1e-12);
  EXPECT_NEAR(o.signedDistance(4, 5, 0, 8.0), 1.1, 1e-12);
}

TEST(Obstacle, CubicMotionIsFollowedNotApproximated)
{
  // A cubic Bezier over t in [0, 3]: control points at x = 0, 3, 3, 0 with
  // y = z = 0. Bernstein at s: x(s) = 3*3*s(1-s)^2 + 3*3*s^2(1-s)
  //                                 = 9 s (1-s). At s = 1/2, x = 2.25.
  // A straight-line reading of the endpoints would say x = 0 there: this is
  // the case two shipped scenarios (curve, loiter) actually use.
  Obstacle o;
  o.name = "c"; o.radius = 0.5;
  o.control_points = {{0, 0, 0, 0}, {3, 0, 0, 1}, {3, 0, 0, 2}, {0, 0, 0, 3}};
  EXPECT_NEAR(o.centreAt(1.5)[0], 2.25, 1e-12);
  EXPECT_NEAR(o.centreAt(0.0)[0], 0.0, 1e-12);
  EXPECT_NEAR(o.centreAt(3.0)[0], 0.0, 1e-12);
  // s = 1/4: 9 * 0.25 * 0.75 = 1.6875
  EXPECT_NEAR(o.centreAt(0.75)[0], 1.6875, 1e-12);
}

TEST(Obstacle, OutsideItsWindowAnObstacleIsAbsentNotParked)
{
  // The wall that stands until t=5 and is then gone. At t=5+ it must not
  // constrain anything: the planner solved a problem in which it is not
  // there, and waiting for it is the behaviour `wall` demonstrates.
  const auto o = moving("W", 0, 0, 0, 1.0, 0, 0, 0, 0.0, 5.0);
  EXPECT_TRUE(o.activeAt(0.0));
  EXPECT_TRUE(o.activeAt(5.0));
  EXPECT_FALSE(o.activeAt(5.001));
  EXPECT_FALSE(o.activeAt(-0.001));
  EXPECT_NEAR(o.signedDistance(0, 0, 0, 2.0), -1.0, 1e-12);
  EXPECT_TRUE(std::isinf(o.signedDistance(0, 0, 0, 6.0)));
  EXPECT_TRUE(std::isinf(o.signedDistance(0, 0, 0, -1.0)));
}

TEST(Obstacle, AColumnIgnoresAltitude)
{
  // A 2D scenario's obstacle is a disc at EVERY height. Drawn at z=1.5, but
  // 100 m up is still inside it -- which is the whole reason the type exists:
  // as a sphere the solver's 2D problem would gain an escape over the top.
  const auto col = moving("c", 0, 0, 1.5, 1.0, 0, 0, 0, 0.0, 10.0, "column");
  EXPECT_NEAR(col.signedDistance(2, 0, 1.5, 0.0), 1.0, 1e-12);
  EXPECT_NEAR(col.signedDistance(2, 0, 100.0, 0.0), 1.0, 1e-12);
  EXPECT_NEAR(col.signedDistance(0, 0, -50.0, 0.0), -1.0, 1e-12);
  // The same geometry as a sphere would be 98.5 m clear at 100 m up.
  const auto ball = moving("s", 0, 0, 1.5, 1.0, 0, 0, 0, 0.0, 10.0, "sphere");
  EXPECT_GT(ball.signedDistance(2, 0, 100.0, 0.0), 90.0);
}

TEST(ClearanceTracker, SubtractsTheVehicleRadius)
{
  // Unit sphere, 0.3 m vehicle. A point 1.25 m out has 0.25 m to the surface
  // and the envelope is 0.3 m wide: 5 cm of penetration.
  ClearanceTracker tr({moving("s", 0, 0, 0, 1.0)}, 0.3);
  const auto s = tr.update(1.25, 0, 0, 0.0);
  EXPECT_NEAR(s.clearance, -0.05, 1e-12);
  EXPECT_EQ(s.nearest, "s");
  ASSERT_EQ(tr.hits().size(), 1u);
}

TEST(ClearanceTracker, NamesTheNearestNotTheFirst)
{
  ClearanceTracker tr({moving("far", 10, 0, 0, 1.0), moving("near", 0, 3, 0, 1.0)}, 0.0);
  EXPECT_EQ(tr.update(0, 0, 0, 0.0).nearest, "near");
  EXPECT_NEAR(tr.update(0, 0, 0, 0.0).clearance, 2.0, 1e-12);
}

TEST(ClearanceTracker, AnAbsentObstacleConstrainsNothing)
{
  // Sitting exactly where a wall used to be, after it has gone.
  ClearanceTracker tr({moving("W", 0, 0, 0, 1.0, 0, 0, 0, 0.0, 5.0)}, 0.3);
  EXPECT_LT(tr.update(0, 0, 0, 1.0).clearance, 0.0);
  tr.reset();
  const auto s = tr.update(0, 0, 0, 7.0);
  EXPECT_TRUE(std::isinf(s.clearance));
  EXPECT_TRUE(tr.hits().empty());
  EXPECT_TRUE(std::isinf(tr.minClearance()));
}

TEST(ClearanceTracker, OneExcursionIsOneHit)
{
  ClearanceTracker tr({moving("s", 0, 0, 0, 1.0, 0, 0, 0, 0.0, 1e6)}, 0.0);
  // Approach along +x from 2.0 to 0.2 (inside from x < 1.0), then back out.
  for (int i = 0; i <= 180; ++i) {tr.update(2.0 - 0.01 * i, 0, 0, 0.1 * i);}
  ASSERT_EQ(tr.hits().size(), 1u) << "180 samples inside one sphere is one collision";
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
  ClearanceTracker tr({moving("s", 0, 0, 0, 1.0)}, 0.0);
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
    fence.push_back(moving("F" + std::to_string(i), 4.0, 0.5 + 0.9 * i, 0.0, 0.9, 0.25));
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
  // (motion ignored) would first hit at 0.5 + 0.9 t = 4 - 1.09087, t = 2.68.
  // The 3.71 above is only right because the fence moved.
}

// ---------------------------------------------------------------------------
// Line of sight. A station is a fixed ground point the vehicle must stay
// VISIBLE from, and an obstacle takes that away by standing between them --
// which it can do while the vehicle is nowhere near it. Every number below is
// derived in the comment beside it; the arithmetic is
//
//     distance( obstacle centre at t, segment[station, vehicle] ) - radius
//
// with the foot of the perpendicular CLAMPED to the segment.
// ---------------------------------------------------------------------------

namespace
{
const std::array<double, 3> ORIGIN_STATION{0.0, 0.0, 0.0};
}

TEST(LineOfSight, ABodyOnTheSightLineBlocksItByItsRadius)
{
  // Station (0,0,0), vehicle (10,0,0), obstacle centred at (5,0,0) r=1. The
  // segment runs straight through the centre, so the distance is 0 and the
  // margin is 0 - 1 = -1: blocked, by one radius.
  const auto o = moving("B", 5, 0, 0, 1.0);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 5.0), -1.0, 1e-12);
}

TEST(LineOfSight, ABodyBesideTheSightLineDoesNotBlockIt)
{
  // Same segment, obstacle at (5,0,3): the perpendicular distance is 3, so
  // the margin is 3 - 1 = 2.
  const auto o = moving("B", 5, 0, 3, 1.0);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 5.0), 2.0, 1e-12);
}

TEST(LineOfSight, ABodyBehindTheStationIsNotBetweenThem)
{
  // Obstacle at (-5,0,0), vehicle at (10,0,0). On the INFINITE line the
  // distance is 0 and it would read as blocked; on the SEGMENT the nearest
  // point is the station itself, 5 m away, so the margin is 5 - 1 = 4.
  //
  // FAILS IF: the foot of the perpendicular is not clamped. That mistake
  // reports the sight line lost to a body the vehicle is flying away from.
  const auto o = moving("B", -5, 0, 0, 1.0);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 5.0), 4.0, 1e-12);
}

TEST(LineOfSight, ABodyBeyondTheVehicleIsNotBetweenThemEither)
{
  // Obstacle at (15,0,0), vehicle at (10,0,0): nearest point on the segment is
  // the vehicle, 5 m away, margin 5 - 1 = 4. The other end of the same clamp.
  const auto o = moving("B", 15, 0, 0, 1.0);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 5.0), 4.0, 1e-12);
}

TEST(LineOfSight, AnInactiveBodyBlocksNothing)
{
  // Dead centre of the sight line, but its window is [0, 10] and we ask at
  // t=11. An obstacle outside its window does not exist, so it cannot occlude:
  // +infinity, not -1.
  const auto o = moving("B", 5, 0, 0, 1.0, 0, 0, 0, 0.0, 10.0);
  EXPECT_EQ(o.losMargin(ORIGIN_STATION, 10, 0, 0, 11.0), INF);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 9.0), -1.0, 1e-12);
}

TEST(LineOfSight, AMovingBodyIsFollowedNotFrozen)
{
  // Crosses the sight line: centre starts at (5,-4,0) and moves +1 m/s in y,
  // so at t=4 it is at (5,0,0) -- on the line, margin -1 -- and at t=0 it is
  // 4 m off it, margin 3.
  const auto o = moving("B", 5, -4, 0, 1.0, 0, 1, 0);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 0.0), 3.0, 1e-12);
  EXPECT_NEAR(o.losMargin(ORIGIN_STATION, 10, 0, 0, 4.0), -1.0, 1e-12);
}

TEST(LineOfSight, NoStationsMeansNoConstraintNotAZeroMargin)
{
  // FAILS IF: an unconstrained scenario reports 0.0, which reads as "only just
  // visible" and would put every scenario one rounding error from a failure.
  ClearanceTracker tr({moving("B", 5, 0, 0, 1.0)}, 0.3);
  const auto s = tr.update(10, 0, 0, 5.0);
  EXPECT_EQ(s.los_margin, INF);
  EXPECT_TRUE(s.los_blocker.empty());
  EXPECT_TRUE(tr.blackouts().empty());
  EXPECT_EQ(tr.minLosMargin(), INF);
}

TEST(LineOfSight, TheWorstStationIsTheOneReported)
{
  // Two stations. From (0,0,0) the body at (5,0,0) is dead on the line:
  // margin -1. From (0,0,10) the segment to (10,0,0) passes the body at
  // distance |(5,0,0) - foot|; the segment direction is (10,0,-10)/sqrt(200),
  // and the foot parameter is ((5,0,-10) . (10,0,-10))/200 = 150/200 = 0.75,
  // giving foot (7.5,0,2.5) and distance sqrt(2.5^2 + 2.5^2) = 3.5355, margin
  // 2.5355. The worst of the two is -1, from station0.
  ClearanceTracker tr({moving("B", 5, 0, 0, 1.0)}, 0.3,
    {{{0.0, 0.0, 0.0}}, {{0.0, 0.0, 10.0}}});
  const auto s = tr.update(10, 0, 0, 5.0);
  EXPECT_NEAR(s.los_margin, -1.0, 1e-12);
  EXPECT_EQ(s.los_station, "station0");
  EXPECT_EQ(s.los_blocker, "B");
}

TEST(LineOfSight, OneExcursionBehindABodyIsOneBlackout)
{
  // The vehicle sits at (10,0,0); the body crosses the sight line at 1 m/s in
  // y from (5,-4,0). It occludes while its centre is within 1 m of the line,
  // that is |y| < 1, so from t=3 to t=5: one blackout, 2 s long, deepest at
  // t=4 where the margin is -1.
  //
  // FAILS IF: the event is per sample. At 100 Hz this pass would be reported
  // as 200 separate losses of sight.
  ClearanceTracker tr({moving("B", 5, -4, 0, 1.0, 0, 1, 0)}, 0.3, {ORIGIN_STATION});
  for (int i = 0; i <= 1000; ++i) {tr.update(10, 0, 0, 0.01 * i);}
  ASSERT_EQ(tr.blackouts().size(), 1u);
  const auto & b = tr.blackouts()[0];
  EXPECT_EQ(b.station, "station0");
  EXPECT_EQ(b.blocker, "B");
  EXPECT_NEAR(b.t, 3.0, 0.011);              // first 10 ms sample past 3.0
  EXPECT_NEAR(b.duration, 2.0, 0.021);
  EXPECT_NEAR(b.depth, 1.0, 1e-3);
  EXPECT_NEAR(tr.minLosMargin(), -1.0, 1e-3);
}

TEST(LineOfSight, SightCanBeLostWithoutGoingAnywhereNearTheObstacle)
{
  // The whole reason this is a separate measure. Vehicle at (10,0,0), body of
  // radius 1 at (5,0,0): the clearance is |(10,0,0)-(5,0,0)| - 1 - 0.3 = 3.7,
  // comfortably clear, while the sight line from the origin is blocked by 1 m.
  // A referee that only scored collision would call this run perfect.
  ClearanceTracker tr({moving("B", 5, 0, 0, 1.0)}, 0.3, {ORIGIN_STATION});
  const auto s = tr.update(10, 0, 0, 5.0);
  EXPECT_NEAR(s.clearance, 3.7, 1e-12);
  EXPECT_TRUE(tr.hits().empty());
  EXPECT_NEAR(s.los_margin, -1.0, 1e-12);
  EXPECT_EQ(tr.blackouts().size(), 1u);
}

TEST(LineOfSight, ResetClearsTheBlackoutsAndTheWorstMargin)
{
  ClearanceTracker tr({moving("B", 5, 0, 0, 1.0)}, 0.3, {ORIGIN_STATION});
  tr.update(10, 0, 0, 5.0);
  ASSERT_EQ(tr.blackouts().size(), 1u);
  tr.reset();
  EXPECT_TRUE(tr.blackouts().empty());
  EXPECT_EQ(tr.minLosMargin(), INF);
  // And the next excursion is a NEW event, not a continuation of the old one.
  tr.update(10, 0, 0, 5.0);
  EXPECT_EQ(tr.blackouts().size(), 1u);
}

TEST(LineOfSight, AColumnWithStationsIsRefusedRatherThanGuessedAt)
{
  // The planner treats every obstacle as a ball for occlusion, so there is no
  // column-vs-sight-line arithmetic on that side to check this one against.
  // A number with nothing to check it against is worse than no number.
  EXPECT_THROW(
    ClearanceTracker({moving("C", 5, 0, 0, 1.0, 0, 0, 0, 0.0, 10.0, "column")},
    0.3, {ORIGIN_STATION}),
    std::runtime_error);
  // Without stations the same column is fine: nothing is asking about sight.
  EXPECT_NO_THROW(
    ClearanceTracker({moving("C", 5, 0, 0, 1.0, 0, 0, 0, 0.0, 10.0, "column")}, 0.3));
}

TEST(LineOfSight, SightLostTwiceIsTwoBlackouts)
{
  // A body that crosses the sight line, clears it, and crosses back. Its
  // centre follows a QUADRATIC in y with control points -4, +14, -4, so
  //     y(s) = -4 + 36 s - 36 s^2,   t = 10 s
  // which runs -4 -> +5 -> -4. The vehicle sits at (10,0,0) and the station is
  // the origin, so the sight line is the x axis and the body (r=1) occludes
  // while |y| < 1. Solving -4 + 36s - 36s^2 = -1 gives s = 0.09175, 0.90825;
  // = +1 gives s = 0.16667, 0.83333. So sight is lost over
  //     t in [0.9175, 1.6667]  and  t in [8.3333, 9.0825]
  // -- two excursions of 0.749 s each. y passes through 0 inside each one
  // (s = 0.12732 and 0.87268), where the margin would be exactly -1. The
  // samples are 10 ms apart, so the grid lands at s = 0.127 instead, where
  // y = -4 + 4.572 - 0.580644 = -0.008644 and the depth is 0.991356. That is
  // the number asserted: what this sampling actually sees, not the limit it
  // approaches.
  //
  // FAILS IF: regaining sight does not clear the per-station flag. Then the
  // second loss is folded into the first and the run reports one blackout that
  // appears to last eight seconds, over a stretch where the station could see
  // the vehicle perfectly well.
  Obstacle o;
  o.name = "X"; o.radius = 1.0;
  o.control_points = {
    ControlPoint{5, -4, 0, 0.0}, ControlPoint{5, 14, 0, 5.0}, ControlPoint{5, -4, 0, 10.0}};
  ClearanceTracker tr({o}, 0.3, {ORIGIN_STATION});
  for (int i = 0; i <= 1000; ++i) {tr.update(10, 0, 0, 0.01 * i);}
  ASSERT_EQ(tr.blackouts().size(), 2u);
  EXPECT_NEAR(tr.blackouts()[0].t, 0.9175, 0.011);
  EXPECT_NEAR(tr.blackouts()[0].duration, 0.749, 0.021);
  EXPECT_NEAR(tr.blackouts()[0].depth, 0.991356, 1e-5);
  EXPECT_NEAR(tr.blackouts()[1].t, 8.3333, 0.011);
  EXPECT_NEAR(tr.blackouts()[1].duration, 0.749, 0.021);
  EXPECT_NEAR(tr.blackouts()[1].depth, 0.991356, 1e-5);
}

TEST(LineOfSight, TwoSeparateHitsAreTwoHits)
{
  // The same shape of check for the clearance side, which had no test for
  // re-entry either: the body above passes THROUGH the vehicle's position
  // twice if the vehicle sits on its path. Vehicle at (5,0,0), body r=1 plus
  // the 0.3 m envelope, so it is a hit while |y| < 1.3.
  Obstacle o;
  o.name = "X"; o.radius = 1.0;
  o.control_points = {
    ControlPoint{5, -4, 0, 0.0}, ControlPoint{5, 14, 0, 5.0}, ControlPoint{5, -4, 0, 10.0}};
  ClearanceTracker tr({o}, 0.3);
  for (int i = 0; i <= 1000; ++i) {tr.update(5, 0, 0, 0.01 * i);}
  EXPECT_EQ(tr.hits().size(), 2u);
}
