// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure per-node wheel between-factor σ_x model, factored out of
// GraphManager::Tick so it is unit-testable without ROS/GTSAM (same pattern
// as rtk_wrongfix_gate.hpp).
//
// WHY DISTANCE-PROPORTIONAL (field incident 2026-08-04): the legacy model
// used a FIXED σ_x per node (0.05 m, sized for a 10 Hz cadence but run at
// 25 Hz) and swapped to a fixed 0.5 m during pivots. Without an accepted GPS
// fix the marginal covariance therefore random-walks at 0.05·√(25·T) — 0.56 m
// after five seconds, and metres within ~2 s during a pivot. At multipath
// corners the RTK wrong-fix gate legitimately drops a couple of fixes exactly
// while FTC pivots, so σ_xy blew past LocalizationGuard's 0.15 m threshold
// and paused mowing although the receiver accuracy never left ~1.1 cm; the
// resulting pause storms burned every dispatch attempt and whole areas were
// skipped. Physically, differential-drive odometry drifts ~1-2 % of the
// distance DRIVEN — a fixed per-node σ overstates the error by orders of
// magnitude whenever the robot is slow or stationary.
//
// Model: σ_x = max(floor, k · dist_reported). `dist_reported` is the RAW
// wheel translation this node (before any slip veto) — during a pivot the
// wheels' phantom forward component IS the reported distance, so a pivot
// coefficient ≥ 1 covers the whole lie while still bounding σ growth to the
// actual (small) per-node distances instead of a fixed half-metre.
// A 10 s GPS gap at 0.2 m/s with k=0.5 now grows σ by only
// 0.004·√250 ≈ 6 cm — LocalizationGuard keeps its meaning for genuine
// minutes-long degradations (RTK → plain GPS) and stops firing on 2 s
// multipath burps.

#pragma once

#include <algorithm>

namespace fusion_graph
{

// Per-node wheel σ_x from the distance the encoders reported this node.
//   dist_reported_m  raw |wheel translation| accumulated this node (pre-veto)
//   pivoting         gyro-gated pivot state (phantom-forward-vx regime)
//   sigma_per_m      σ as a fraction of reported distance (straight driving)
//   pivot_sigma_per_m  same during pivots — must be ≥ 1 so σ covers a fully
//                      fictional reported translation
//   floor_m          lower bound (encoder quantisation / stationary jitter)
inline double DistanceScaledWheelSigmaX(double dist_reported_m,
                                        bool pivoting,
                                        double sigma_per_m,
                                        double pivot_sigma_per_m,
                                        double floor_m)
{
  const double k = pivoting ? pivot_sigma_per_m : sigma_per_m;
  return std::max(floor_m, k * dist_reported_m);
}

}  // namespace fusion_graph
