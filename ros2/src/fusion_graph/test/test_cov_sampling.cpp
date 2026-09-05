// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Published-covariance sampling (field incident 2026-08-10).
//
// Two related failure modes made LocalizationGuard pause blade-on mowing on
// PHANTOM σ spikes while the receiver was RTK-Fixed at 1-2 cm the whole time:
//
//  1. GPS epochs and node creation are not phase-locked, so some tip nodes
//     carry no GnssLeverArmFactor yet when the throttled marginal samples
//     them. During slope-slip storms (adaptive wheel σ inflated to
//     0.2-1.5 m/node) such a tip marginal read 0.2-0.4 m although the
//     neighbouring GPS-pinned nodes were centimetre-tight.
//     → Fix: sample the newest GPS-carrying node within cov_gps_node_max_lag
//       of the tip; beyond the lag fall back to the tip so a REAL outage
//       still shows honestly growing σ.
//
//  2. The refresh used to run every Nth NODE. The moment the guard halts the
//     robot, the stationary throttle slows nodes to 1/5 s and the refresh to
//     ~50 s — a phantom spike latched at pause-entry stayed published long
//     after the true marginal collapsed, and the σ<resume gate self-extended
//     every pause to ~1 min.
//     → Fix: wall-clock cadence (cov_update_period_s) + RefreshLatestCovariance()
//       on node-less ticks.

#include <cmath>
#include <optional>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

fg::GraphParams MakeParams()
{
  fg::GraphParams gp;
  // Half the stepped 0.1 s cadence: Step() advances now_s by 0.1*(i+1),
  // whose float rounding occasionally lands a hair BELOW an exact 0.1 s
  // period and makes Tick() skip the node (rate gate `dt < node_period_s`).
  gp.node_period_s = 0.05;
  // Deliberately huge along-track random walk so a node without a GPS unary
  // has an unmistakably large marginal — the test then cleanly separates
  // "sampled the GPS node" from "sampled the tip". Steps are 0.5 m/s · 0.1 s
  // = 0.05 m, so k = 4.5 m/sqrt(m) gives sigma_x = 4.5*sqrt(0.05) = 1.0 m per
  // node (creep floor off to keep it exact).
  gp.wheel_sigma_x_per_sqrt_m = 4.5;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_creep_speed_mps = 0.0;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_node_period_s = 0.0;  // a node every tick
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 0.0;
  gp.cov_update_period_s = 0.0;  // refresh on every opportunity
  gp.cov_gps_node_max_lag = 20;
  return gp;
}

// One forward tick; queues a GPS unary at the wheel-predicted position when
// with_gps is set. Returns the tick's output (node created every tick here).
std::optional<fg::TickOutput> Step(
    fg::GraphManager& gm, int i, double vx, bool with_gps, double gps_sigma = 0.01)
{
  const double dt = 0.1;
  gm.AddWheelTwist(vx, 0.0, 0.0, dt);
  gm.AddGyroDelta(0.0, dt);
  if (with_gps)
  {
    gm.QueueGnss(vx * dt * (i + 1), 0.0, gps_sigma);
  }
  return gm.Tick(dt * (i + 1));
}

}  // namespace

// A tip node without a GPS unary must NOT flash the (huge) wheel σ into the
// published covariance while a GPS-pinned node sits within the lag window.
TEST(CovSampling, GpsCarryingNodeSampledInsteadOfBareTip)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  // GPS on every node — baseline σ is centimetre-scale.
  std::optional<fg::TickOutput> out;
  int i = 0;
  for (; i < 10; ++i)
    out = Step(gm, i, 0.5, /*with_gps=*/true);
  ASSERT_TRUE(out.has_value());
  const double sigma_baseline = std::sqrt(out->covariance(0, 0));
  EXPECT_LT(sigma_baseline, 0.1);

  // Three GPS-less nodes (epoch phase gap). Published σ must stay pinned —
  // the sampler follows the newest GPS-carrying node (lag 3 ≤ 20).
  for (int k = 0; k < 3; ++k, ++i)
    out = Step(gm, i, 0.5, /*with_gps=*/false);
  ASSERT_TRUE(out.has_value());
  EXPECT_LT(std::sqrt(out->covariance(0, 0)), 0.1)
      << "a GPS-less tip node must not publish the raw wheel sigma";
}

// Regression guard for the FIX itself: with the lag disabled the same
// sequence MUST show the large tip σ — proving the first assertion above
// really exercises the GPS-node sampling and not some other damping.
TEST(CovSampling, LagZeroRestoresTipSampling)
{
  auto params = MakeParams();
  params.cov_gps_node_max_lag = 0;
  fg::GraphManager gm(params);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  std::optional<fg::TickOutput> out;
  int i = 0;
  for (; i < 10; ++i)
    out = Step(gm, i, 0.5, /*with_gps=*/true);
  for (int k = 0; k < 3; ++k, ++i)
    out = Step(gm, i, 0.5, /*with_gps=*/false);
  ASSERT_TRUE(out.has_value());
  EXPECT_GT(std::sqrt(out->covariance(0, 0)), 0.5)
      << "with the lag disabled the bare tip marginal must dominate";
}

// A REAL GPS outage (gap longer than the lag) must fall back to the tip and
// show honestly growing σ — the guard's protection against genuine
// degradation stays intact.
TEST(CovSampling, FallsBackToTipBeyondLag)
{
  auto params = MakeParams();
  params.cov_gps_node_max_lag = 2;
  fg::GraphManager gm(params);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  std::optional<fg::TickOutput> out;
  int i = 0;
  for (; i < 10; ++i)
    out = Step(gm, i, 0.5, /*with_gps=*/true);
  // 5 GPS-less nodes > lag 2 → tip sampling resumes → σ must grow.
  for (int k = 0; k < 5; ++k, ++i)
    out = Step(gm, i, 0.5, /*with_gps=*/false);
  ASSERT_TRUE(out.has_value());
  EXPECT_GT(std::sqrt(out->covariance(0, 0)), 0.5)
      << "beyond the lag a genuine outage must show growing sigma";
}

// RefreshLatestCovariance must un-freeze a stale latched σ without a node
// being created (the stationary-pause path).
TEST(CovSampling, RefreshLatestCovarianceUnfreezesStaleSigma)
{
  auto params = MakeParams();
  params.cov_update_period_s = 100.0;  // only the very first tick refreshes
  fg::GraphManager gm(params);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  // Tick 1: GPS-less → the one-and-only refresh samples the bare tip and
  // latches a large σ (this emulates the phantom spike at pause-entry).
  auto out = Step(gm, 0, 0.5, /*with_gps=*/false);
  ASSERT_TRUE(out.has_value());
  ASSERT_GT(std::sqrt(out->covariance(0, 0)), 0.5);

  // Tick 2: GPS arrives, but the wall-clock period is not due — the
  // published σ stays stale-large (this is the frozen latch).
  out = Step(gm, 1, 0.5, /*with_gps=*/true);
  ASSERT_TRUE(out.has_value());
  ASSERT_GT(std::sqrt(out->covariance(0, 0)), 0.5);

  // The node-less-tick refresh path, called past the period: must recompute
  // at the GPS-carrying node and collapse the published σ.
  gm.RefreshLatestCovariance(0.2 + 200.0);
  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  EXPECT_LT(std::sqrt(snap->covariance(0, 0)), 0.1)
      << "the stale latched sigma must collapse once the refresh runs";
}
