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
  private enum State{NO_BALLS, ONE_BALL, ONE_PLUS_ONE_BALL, PRIMED, TWO_BALLS, FULL};
  private State state;
  
  
  /** Creates a new QueueBalls. */
  public QueueBalls(BooleanProvider booleanProvider) {
    this.booleanProvider = booleanProvider;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder, tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateState();
    System.out.println(state);
    switch(state) {
      case ONE_BALL:
        runFeeder();
        if(feeder.hasBall() && tower.bottomHasBall()){
          state = State.ONE_PLUS_ONE_BALL;
        }
        break;
      case ONE_PLUS_ONE_BALL:
        runFeederSlowly();
        if(!feeder.hasBall()){
          state = State.PRIMED;
        }
        break;
      case PRIMED:
      case FULL:
        stopFeederAndTower();
        break;
      default:
        stopFeederAndTower();
        updateState();
        break;
      case NO_BALLS:
        runFeederAndTower();
        break;
    }
  }

  private void updateState() {
    if (!feeder.hasBall() && !tower.bottomHasBall()) {
      state = State.NO_BALLS;
    } else if (tower.bottomHasBall()) {
      state = State.ONE_BALL;
    }
  }

  private void runFeederAndTower() {
    feeder.run(1);
    tower.run(1);
  }

  private void runFeeder() {
    tower.stop();
    feeder.run(0.5);
  }

  private void runFeederSlowly() {
    tower.stop();
    feeder.run(0.1);
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
