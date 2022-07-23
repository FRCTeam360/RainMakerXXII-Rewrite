// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Tower;
import frc.robot.utils.BooleanProvider;


public class QueueBalls extends CommandBase {
  BooleanProvider booleanProvider;
  Feeder feeder = Feeder.getInstance();
  Tower tower = Tower.getInstance();
  private enum State{RUN_BOTH, RUN_FEEDER, STOP_BOTH};
  private State state;
  
  /** Creates a new QueueBalls. */
  public QueueBalls(BooleanProvider booleanProvider) {
    this.booleanProvider = booleanProvider;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder, tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateState();
    System.out.println(state);
    switch(state) {
      case RUN_FEEDER:
        runFeeder();
        break;
      case STOP_BOTH:
      default:
        stopFeederAndTower();
        break;
      case RUN_BOTH:
        runFeederAndTower();
        break;
    }
  }

  private void updateState() {
    if (booleanProvider.getBoolean()) {
      state = State.RUN_BOTH;
    } else if (feeder.hasBall() && tower.bottomHasBall()) {
      state = State.STOP_BOTH;
    } else if (!feeder.hasBall() && tower.bottomHasBall()) {
      state = State.RUN_FEEDER;
    } else {
      state = State.STOP_BOTH;
    }
  }

  private void runFeederAndTower() {
    feeder.run(1);
    tower.run(1);
  }

  private void runFeeder() {
    tower.stop();
    feeder.run(1);
  }

  private void stopFeederAndTower() {
    tower.stop();
    feeder.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stopFeederAndTower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
