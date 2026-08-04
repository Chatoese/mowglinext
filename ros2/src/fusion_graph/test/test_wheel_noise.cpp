// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Regression tests for the distance-proportional wheel σ_x model
// (wheel_noise.hpp). Motivating field incident 2026-08-04: the legacy FIXED
// per-node σ (0.05 m, 0.5 m during pivots) let the marginal covariance
// random-walk to 0.5-1.8 m within a couple of gate-rejected GPS fixes, so
// LocalizationGuard paused mowing while the receiver accuracy sat at 1.1 cm
// and the pause storms burned every area dispatch attempt.

#include "fusion_graph/wheel_noise.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
constexpr double kPerM = 0.5;       // straight-driving coefficient
constexpr double kPivotPerM = 3.0;  // pivot coefficient (covers phantom vx)
constexpr double kFloor = 0.002;    // m
}  // namespace

// Stationary: no reported travel → σ pinned to the floor, NOT a fixed 5 cm.
// This is what stops multi-second GPS gaps from ballooning the marginal
// covariance while the robot is standing still (guard pause at the dock).
TEST(WheelNoise, StationaryStaysAtFloor)
{
  EXPECT_DOUBLE_EQ(fg::DistanceScaledWheelSigmaX(0.0, false, kPerM, kPivotPerM, kFloor), kFloor);
  EXPECT_DOUBLE_EQ(fg::DistanceScaledWheelSigmaX(0.0, true, kPerM, kPivotPerM, kFloor), kFloor);
}

// Mowing speed, 25 Hz node cadence: 0.2 m/s → 8 mm reported per node.
// σ = 4 mm per node → a 10 s GPS gap grows σ by ~0.004·√250 ≈ 6 cm, safely
// below LocalizationGuard's 0.15 m pause threshold. The legacy fixed model
// produced 0.05·√250 ≈ 0.79 m for the same gap.
TEST(WheelNoise, DrivingSigmaIsFractionOfTravel)
{
  const double per_node = 0.2 * 0.04;  // m
  const double sigma = fg::DistanceScaledWheelSigmaX(per_node, false, kPerM, kPivotPerM, kFloor);
  EXPECT_DOUBLE_EQ(sigma, kPerM * per_node);
  EXPECT_LT(sigma, 0.005);
}

// Pivot: the coefficient must cover a FULLY fictional reported translation
// (phantom forward vx, 2026-05-27 stuck-rotate incident) — σ ≥ the reported
// distance itself.
TEST(WheelNoise, PivotSigmaCoversPhantomTranslation)
{
  const double phantom = 0.1 * 0.04;  // wheels lie 0.1 m/s forward at 25 Hz
  const double sigma = fg::DistanceScaledWheelSigmaX(phantom, true, kPerM, kPivotPerM, kFloor);
  EXPECT_GE(sigma, phantom) << "pivot sigma must cover the whole phantom delta";
  EXPECT_DOUBLE_EQ(sigma, kPivotPerM * phantom);
  // And bounded: nowhere near the legacy fixed 0.5 m/node that exploded the
  // marginal within a two-second pivot.
  EXPECT_LT(sigma, 0.05);
}

// Floor wins whenever the proportional term is smaller (creeping speeds).
TEST(WheelNoise, FloorDominatesAtCreepingSpeed)
{
  const double tiny = 0.001;  // 1 mm reported
  EXPECT_DOUBLE_EQ(fg::DistanceScaledWheelSigmaX(tiny, false, kPerM, kPivotPerM, kFloor), kFloor);
}
