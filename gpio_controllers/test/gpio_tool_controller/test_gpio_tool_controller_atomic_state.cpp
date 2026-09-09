// Copyright (c) 2025, b»robotized by Stogl Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Regression tests for the (action, transition) race: a goal thread starting a new action and
// the RT update() thread re-affirming the idle scan could interleave so the RT thread's write
// clobbers the goal thread's, wedging the tool permanently (see the client bug report this
// fixes). The pair is now packed into one atomic and every write goes through a
// compare_exchange against a value the writer just read, instead of a plain store.
//
// These tests exercise try_advance() - the same compare_exchange pattern used at every write
// site in gpio_tool_controller.cpp - directly. No threads or sleeps are needed: "a concurrent
// writer got there first" is simulated by simply overwriting the packed state between reading
// the snapshot and attempting the compare_exchange, which is deterministic and instant.

#include "test_gpio_tool_controller.hpp"

// ---------------------------------------------------------------------------
// Nothing else touched the state since the snapshot was read - the compare_exchange succeeds.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, UncontestedAdvanceSucceeds)
{
  controller_->set_state(ToolAction::IDLE, GPIOToolTransition::IDLE);
  uint16_t expected = controller_->get_packed_state();

  const bool advanced =
    controller_->try_advance(expected, ToolAction::ENGAGING, GPIOToolTransition::SET_BEFORE_COMMAND);

  EXPECT_TRUE(advanced);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// Simulates the exact race from the bug report: an RT tick reads (IDLE, IDLE) intending to
// re-affirm it (the idle-scan case), but a goal thread starts ENGAGING before the tick's
// compare_exchange lands. The tick must back off instead of clobbering the goal thread's write.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, ConcurrentChangeIsNotClobbered)
{
  controller_->set_state(ToolAction::IDLE, GPIOToolTransition::IDLE);
  uint16_t stale_snapshot = controller_->get_packed_state();  // what the RT tick "read"

  // The goal thread gets there first.
  controller_->set_state(ToolAction::ENGAGING, GPIOToolTransition::SET_BEFORE_COMMAND);

  // The RT tick's compare_exchange, still holding the stale snapshot, must fail.
  uint16_t expected = stale_snapshot;
  const bool advanced =
    controller_->try_advance(expected, ToolAction::IDLE, GPIOToolTransition::IDLE);

  EXPECT_FALSE(advanced);
  // compare_exchange_strong updates `expected` to the actual current value on failure.
  EXPECT_EQ(expected, controller_->get_packed_state());
  // The goal thread's write survives untouched.
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}
