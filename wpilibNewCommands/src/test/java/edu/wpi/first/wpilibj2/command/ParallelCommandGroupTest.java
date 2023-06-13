// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.Test;

class ParallelCommandGroupTest extends CommandTestBase
    implements MultiCompositionTestBase<ParallelCommandGroup> {
  @Test
  void parallelGroupScheduleTest() {
    try (CommandScheduler scheduler = new CommandScheduler()) {
      MockCommandHolder command1Holder = new MockCommandHolder(true);
      Command command1 = command1Holder.getMock();
      MockCommandHolder command2Holder = new MockCommandHolder(true);
      Command command2 = command2Holder.getMock();

      Command group = new ParallelCommandGroup(command1, command2);

      scheduler.schedule(group);

      verify(command1).initialize();
      verify(command2).initialize();

      command1Holder.setFinished(true);
      scheduler.run();
      command2Holder.setFinished(true);
      scheduler.run();

      verify(command1).execute();
      verify(command1).end(false);
      verify(command2, times(2)).execute();
      verify(command2).end(false);

      assertFalse(scheduler.isScheduled(group));
    }
  }

  @Test
  void parallelGroupInterruptTest() {
    try (CommandScheduler scheduler = new CommandScheduler()) {
      MockCommandHolder command1Holder = new MockCommandHolder(true);
      Command command1 = command1Holder.getMock();
      MockCommandHolder command2Holder = new MockCommandHolder(true);
      Command command2 = command2Holder.getMock();

      Command group = new ParallelCommandGroup(command1, command2);

      scheduler.schedule(group);

      command1Holder.setFinished(true);
      scheduler.run();
      scheduler.run();
      scheduler.cancel(group);

      verify(command1).execute();
      verify(command1).end(false);
      verify(command1, never()).end(true);

      verify(command2, times(2)).execute();
      verify(command2, never()).end(false);
      verify(command2).end(true);

      assertFalse(scheduler.isScheduled(group));
    }
  }

  @Test
  void notScheduledCancelTest() {
    try (CommandScheduler scheduler = new CommandScheduler()) {
      MockCommandHolder command1Holder = new MockCommandHolder(true);
      Command command1 = command1Holder.getMock();
      MockCommandHolder command2Holder = new MockCommandHolder(true);
      Command command2 = command2Holder.getMock();

      Command group = new ParallelCommandGroup(command1, command2);

      assertDoesNotThrow(() -> scheduler.cancel(group));
    }
  }

  @Test
  void parallelGroupRequirementTest() {
    Subsystem system1 = new Subsystem() {};
    Subsystem system2 = new Subsystem() {};
    Subsystem system3 = new Subsystem() {};
    Subsystem system4 = new Subsystem() {};

    try (CommandScheduler scheduler = new CommandScheduler()) {
      MockCommandHolder command1Holder = new MockCommandHolder(true, system1, system2);
      Command command1 = command1Holder.getMock();
      MockCommandHolder command2Holder = new MockCommandHolder(true, system3);
      Command command2 = command2Holder.getMock();
      MockCommandHolder command3Holder = new MockCommandHolder(true, system3, system4);
      Command command3 = command3Holder.getMock();

      Command group = new ParallelCommandGroup(command1, command2);

      scheduler.schedule(group);
      scheduler.schedule(command3);

      assertFalse(scheduler.isScheduled(group));
      assertTrue(scheduler.isScheduled(command3));
    }
  }

  @Test
  void parallelGroupRequirementErrorTest() {
    Subsystem system1 = new Subsystem() {};
    Subsystem system2 = new Subsystem() {};
    Subsystem system3 = new Subsystem() {};

    MockCommandHolder command1Holder = new MockCommandHolder(true, system1, system2);
    Command command1 = command1Holder.getMock();
    MockCommandHolder command2Holder = new MockCommandHolder(true, system2, system3);
    Command command2 = command2Holder.getMock();

    assertThrows(
        IllegalArgumentException.class, () -> new ParallelCommandGroup(command1, command2));
  }

  @Override
  public ParallelCommandGroup compose(Command... members) {
    return new ParallelCommandGroup(members);
  }
}
