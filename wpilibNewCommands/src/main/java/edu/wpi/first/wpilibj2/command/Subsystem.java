// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * A robot subsystem. Subsystems are the basic unit of robot organization in the Command-based
 * framework; they encapsulate low-level hardware objects (motor controllers, sensors, etc.) and
 * provide methods through which they can be used by {@link Command}s. Subsystems are used by the
 * {@link CommandScheduler}'s resource management system to ensure multiple robot actions are not
 * "fighting" over the same hardware; Commands that use a subsystem should include that subsystem in
 * their {@link Command#getRequirements()} method, and resources used within a subsystem should
 * generally remain encapsulated and not be shared by other parts of the robot.
 *
 * <p>Subsystems are automatically registered with the default scheduler in order for the {@link
 * Subsystem#periodic()} method to be called.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class Subsystem implements Sendable, CommandMutex {
  /** Constructor. */
  public Subsystem() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    SendableRegistry.addLW(this, name, name);
    CommandScheduler.getInstance().registerSubsystem(this);
    CommandScheduler.getInstance().addPeriodic(this::periodic);
    CommandScheduler.getInstance().addSimPeriodic(this::simulationPeriodic);
  }

  /**
   * This method is called periodically by the {@link CommandScheduler}. Useful for updating
   * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
   * to be consistent within their own codebases about which responsibilities will be handled by
   * Commands, and which will be handled here.
   */
  public void periodic() {}

  /**
   * This method is called periodically by the {@link CommandScheduler}. Useful for updating
   * subsystem-specific state that needs to be maintained for simulations, such as for updating
   * {@link edu.wpi.first.wpilibj.simulation} classes and setting simulated sensor readings.
   */
  public void simulationPeriodic() {}

  /**
   * Sets the default {@link Command} of the subsystem. The default command will be automatically
   * scheduled when no other commands are scheduled that require the subsystem. Default commands
   * should generally not end on their own, i.e. their {@link Command#isFinished()} method should
   * always return false. Will automatically register this subsystem with the {@link
   * CommandScheduler}.
   *
   * @param defaultCommand the default command to associate with this subsystem
   */
  public void setDefaultCommand(Command defaultCommand) {
    CommandScheduler.getInstance().setDefaultCommand(this, defaultCommand);
  }

  /**
   * Removes the default command for the subsystem. This will not cancel the default command if it
   * is currently running.
   */
  public void removeDefaultCommand() {
    CommandScheduler.getInstance().removeDefaultCommand(this);
  }

  /**
   * Gets the default command for this subsystem. Returns null if no default command is currently
   * associated with the subsystem.
   *
   * @return the default command associated with this subsystem
   */
  public Command getDefaultCommand() {
    return CommandScheduler.getInstance().getDefaultCommand(this);
  }

  /**
   * Returns the command currently running on this subsystem. Returns null if no command is
   * currently scheduled that requires this subsystem.
   *
   * @return the scheduled command currently requiring this subsystem
   */
  public Command getCurrentCommand() {
    return CommandScheduler.getInstance().requiring(this);
  }

  /**
   * Gets the name of this Subsystem.
   *
   * @return Name
   */
  public String getName() {
    return SendableRegistry.getName(this);
  }

  /**
   * Sets the name of this Subsystem.
   *
   * @param name name
   */
  public void setName(String name) {
    SendableRegistry.setName(this, name);
  }

  /**
   * Gets the subsystem name of this Subsystem.
   *
   * @return Subsystem name
   */
  public String getSubsystem() {
    return SendableRegistry.getSubsystem(this);
  }

  /**
   * Sets the subsystem name of this Subsystem.
   *
   * @param subsystem subsystem name
   */
  public void setSubsystem(String subsystem) {
    SendableRegistry.setSubsystem(this, subsystem);
  }

  /**
   * Registers this subsystem with the {@link CommandScheduler}, allowing its {@link
   * Subsystem#periodic()} method to be called when the scheduler runs.
   */
  public void register() {
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Constructs a command that runs an action once and finishes. Requires this subsystem.
   *
   * @param action the action to run
   * @return the command
   * @see InstantCommand
   */
  public Command runOnce(Runnable action) {
    return Commands.runOnce(action, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted. Requires this
   * subsystem.
   *
   * @param action the action to run
   * @return the command
   * @see RunCommand
   */
  public Command run(Runnable action) {
    return Commands.run(action, this);
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted. Requires this subsystem.
   *
   * @param start the action to run on start
   * @param end the action to run on interrupt
   * @return the command
   * @see StartEndCommand
   */
  public Command startEnd(Runnable start, Runnable end) {
    return Commands.startEnd(start, end, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action. Requires this subsystem.
   *
   * @param run the action to run every iteration
   * @param end the action to run on interrupt
   * @return the command
   */
  public Command runEnd(Runnable run, Runnable end) {
    return Commands.runEnd(run, end, this);
  }

  /**
   * Associates a {@link Sendable} with this Subsystem. Also update the child's name.
   *
   * @param name name to give child
   * @param child sendable
   */
  public void addChild(String name, Sendable child) {
    SendableRegistry.addLW(child, getSubsystem(), name);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
    builder.addStringProperty(
        ".default",
        () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
        null);
    builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
    builder.addStringProperty(
        ".command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
        null);
  }
}
