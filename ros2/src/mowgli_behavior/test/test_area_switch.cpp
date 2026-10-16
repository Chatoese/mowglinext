// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_area_switch.cpp
 * @brief Operator area switch while mowing (AreaSwitchGuard in main_tree.xml).
 *
 * Field 2026-10-16: pressing "Mow" on an area in the GUI while the robot was
 * already mowing did nothing visible. ~/start_in_area parks the request in
 * BTContext::target_area_index and sets current_command = 1, which is already
 * 1 mid-mow — so the request waited until the current area finished and
 * GetNextUnmowedArea re-entered. Three "StartInArea: received area=2" lines,
 * no "targeted run" line.
 *
 * The fix wraps the per-area work (MowOrSkipArea) in a ReactiveSequence that
 * re-checks IsAreaSwitchRequested every tick, halts the in-flight area when a
 * request arrives, and lets the AreaLoop re-cycle into GetNextUnmowedArea —
 * the single consumer of the request.
 *
 * Two layers:
 *   (a) behavioural — the guard shape with stand-ins for GetNextUnmowedArea
 *       (consumer) and MowOrSkipArea (long-running area work);
 *   (b) structural — the real main_tree.xml keeps that shape.
 */

#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsAreaSwitchRequested;

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

namespace
{

/// Observations shared with the stand-in nodes.
struct Probe
{
  int area_starts{0};
  int area_halts{0};
  int switches{0};
  /// Area chosen by the last ConsumeRequest tick (-1 = normal iteration).
  int selected{-2};
};

/// Long-running stand-in for MowOrSkipArea: RUNNING until halted.
class MockMowArea : public BT::StatefulActionNode
{
public:
  MockMowArea(const std::string& name, const BT::NodeConfig& config, Probe* probe)
      : BT::StatefulActionNode(name, config), probe_(probe)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    probe_->area_starts++;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    probe_->area_halts++;
  }

private:
  Probe* probe_;
};

/// The AreaLoop shape from main_tree.xml: the consumer sits BEFORE the guard,
/// MowOrSkipArea runs under the reactive guard, SwitchArea is the fallback.
const char* const kTreeXml = R"(
  <root BTCPP_format="4">
    <BehaviorTree ID="Main">
      <Repeat num_cycles="10">
        <Sequence name="MowOneArea">
          <ConsumeRequest/>
          <Fallback name="MowAreaOrSwitch">
            <ReactiveSequence name="AreaSwitchGuard">
              <Inverter><IsAreaSwitchRequested/></Inverter>
              <MockMowArea/>
            </ReactiveSequence>
            <Sequence name="SwitchArea">
              <IsAreaSwitchRequested/>
              <SwitchProbe/>
            </Sequence>
          </Fallback>
        </Sequence>
      </Repeat>
    </BehaviorTree>
  </root>
)";

struct Harness
{
  std::shared_ptr<BTContext> ctx;
  Probe probe;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
};

std::unique_ptr<Harness> MakeHarness(const std::string& node_name)
{
  auto h = std::make_unique<Harness>();
  h->ctx = std::make_shared<BTContext>();
  h->ctx->node = rclcpp::Node::make_shared(node_name);
  h->ctx->current_command = 1;  // COMMAND_START — mowing

  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", h->ctx);

  Probe* probe = &h->probe;
  auto ctx = h->ctx;
  h->factory.registerNodeType<IsAreaSwitchRequested>("IsAreaSwitchRequested");
  h->factory.registerNodeType<MockMowArea>("MockMowArea", probe);
  // Stand-in for GetNextUnmowedArea's consume-once handling of the request.
  h->factory.registerSimpleAction("ConsumeRequest",
                                  [probe, ctx](BT::TreeNode&)
                                  {
                                    probe->selected = ctx->target_area_index.value_or(-1);
                                    ctx->target_area_index.reset();
                                    return BT::NodeStatus::SUCCESS;
                                  });
  h->factory.registerSimpleAction("SwitchProbe",
                                  [probe](BT::TreeNode&)
                                  {
                                    probe->switches++;
                                    return BT::NodeStatus::SUCCESS;
                                  });
  h->tree = h->factory.createTreeFromText(kTreeXml, blackboard);
  return h;
}

std::string ReadMainTree()
{
  std::ifstream f(MOWGLI_MAIN_TREE_PATH);
  EXPECT_TRUE(f.is_open()) << "Cannot open " << MOWGLI_MAIN_TREE_PATH;
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Text between the opening tag containing @p open_marker and the next
/// occurrence of @p close_tag after it.
std::string Span(const std::string& xml,
                 const std::string& open_marker,
                 const std::string& close_tag)
{
  const auto begin = xml.find(open_marker);
  if (begin == std::string::npos)
  {
    return {};
  }
  const auto end = xml.find(close_tag, begin);
  if (end == std::string::npos)
  {
    return {};
  }
  return xml.substr(begin, end + close_tag.size() - begin);
}

}  // namespace

// ---------------------------------------------------------------------------
// (a) Behavioural
// ---------------------------------------------------------------------------

TEST(IsAreaSwitchRequestedTest, ReflectsThePendingRequestWithoutConsumingIt)
{
  auto h = MakeHarness("test_area_switch_cond");
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsAreaSwitchRequested>("IsAreaSwitchRequested");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", h->ctx);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><IsAreaSwitchRequested/></BehaviorTree></root>)",
      blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  h->ctx->target_area_index = 2;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
      << "GetNextUnmowedArea is the single consumer — the condition must not consume";
  EXPECT_EQ(h->ctx->target_area_index.value_or(-1), 2);
}

// No request: the area keeps mowing, nothing is halted.
TEST(AreaSwitchTest, WithoutARequestTheAreaIsNeverInterrupted)
{
  auto h = MakeHarness("test_area_switch_idle");
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_EQ(h->tree.tickOnce(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(h->probe.area_starts, 1);
  EXPECT_EQ(h->probe.area_halts, 0);
  EXPECT_EQ(h->probe.switches, 0);
  EXPECT_EQ(h->probe.selected, -1);
}

// The field bug: a request arriving MID-MOW halts the current area and the
// loop re-cycles into the consumer, which selects the operator's area.
TEST(AreaSwitchTest, RequestMidMowPreemptsTheCurrentArea)
{
  auto h = MakeHarness("test_area_switch_preempt");
  ASSERT_EQ(h->tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(h->probe.area_starts, 1);

  h->ctx->target_area_index = 2;  // ~/start_in_area while mowing
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(h->tree.tickOnce(), BT::NodeStatus::RUNNING);
  }

  EXPECT_EQ(h->probe.area_halts, 1) << "the in-flight area must be halted exactly once";
  EXPECT_EQ(h->probe.switches, 1) << "SwitchArea (blade off, stop) must run exactly once";
  EXPECT_EQ(h->probe.selected, 2) << "the loop must re-select the operator's area";
  EXPECT_FALSE(h->ctx->target_area_index.has_value()) << "the request is consumed once";
  EXPECT_EQ(h->probe.area_starts, 2) << "mowing continues in the requested area";
}

// A request already pending when the loop starts (GUI "Mow" from idle) is
// consumed BEFORE the guard is evaluated, so it never preempts and never
// spins SwitchArea.
TEST(AreaSwitchTest, RequestPendingAtLoopStartDoesNotPreempt)
{
  auto h = MakeHarness("test_area_switch_initial");
  h->ctx->target_area_index = 1;
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(h->tree.tickOnce(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(h->probe.selected, 1);
  EXPECT_EQ(h->probe.area_starts, 1);
  EXPECT_EQ(h->probe.area_halts, 0);
  EXPECT_EQ(h->probe.switches, 0);
}

// ---------------------------------------------------------------------------
// (b) Structural — the real tree keeps the shape tested above.
// ---------------------------------------------------------------------------

TEST(AreaSwitchTest, MainTreeAreaLoopHasTheSwitchGuard)
{
  const std::string xml = ReadMainTree();
  ASSERT_FALSE(xml.empty());

  const std::string loop = Span(xml, "<Repeat num_cycles=\"100\" name=\"AreaLoop\">", "</Repeat>");
  ASSERT_FALSE(loop.empty()) << "AreaLoop not found in main_tree.xml";

  const auto consumer = loop.find("<GetNextUnmowedArea");
  const auto guard = loop.find("<ReactiveSequence name=\"AreaSwitchGuard\">");
  ASSERT_NE(consumer, std::string::npos);
  ASSERT_NE(guard, std::string::npos) << "AreaSwitchGuard missing from AreaLoop";
  EXPECT_LT(consumer, guard)
      << "GetNextUnmowedArea must run BEFORE the guard, or a request pending at "
         "loop start would preempt forever without ever being consumed";

  // The guard's first child is the inverted request check, and the area work
  // (MowOrSkipArea) is inside the guard.
  const auto first_child = loop.find('<', loop.find('>', guard) + 1);
  EXPECT_EQ(loop.compare(first_child,
                         std::string("<Inverter><IsAreaSwitchRequested/></Inverter>").size(),
                         "<Inverter><IsAreaSwitchRequested/></Inverter>"),
            0)
      << "AreaSwitchGuard must start with <Inverter><IsAreaSwitchRequested/></Inverter>";
  const auto work = loop.find("<Fallback name=\"MowOrSkipArea\">");
  const auto guard_end = loop.find("<Sequence name=\"SwitchArea\">");
  ASSERT_NE(work, std::string::npos);
  ASSERT_NE(guard_end, std::string::npos) << "SwitchArea branch missing";
  EXPECT_GT(work, guard);
  EXPECT_LT(work, guard_end);

  // SAFETY: the switch branch only runs on a real request and turns the blade
  // off before the next area is selected.
  const std::string sw = Span(loop, "<Sequence name=\"SwitchArea\">", "</Sequence>");
  const auto sw_cond = sw.find("<IsAreaSwitchRequested/>");
  const auto sw_blade = sw.find("<SetMowerEnabled enabled=\"false\"/>");
  ASSERT_NE(sw_cond, std::string::npos) << "SwitchArea must be gated on IsAreaSwitchRequested";
  ASSERT_NE(sw_blade, std::string::npos) << "SwitchArea must switch the blade off";
  EXPECT_LT(sw_cond, sw_blade);
  EXPECT_NE(sw.find("<StopMoving/>"), std::string::npos);
}
