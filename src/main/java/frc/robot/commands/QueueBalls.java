// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Tower;


public class QueueBalls extends CommandBase {
  Feeder feeder = new Feeder();
  Tower tower = new Tower();
  private enum State{NO_BALLS, BALL_IN_BOTTOM, BALL_IN_BOTH};
  private State state;
  
  /** Creates a new QueueBalls. */
  public QueueBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder, tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case BALL_IN_BOTTOM:
        moveUnoBall();
        break;
      case BALL_IN_BOTH:
        moveUnoBallPart2();
        break;
      case NO_BALLS:
      default:
        break;
    }
    updateState();

  }

  private void updateState() {
    if (feeder.hasBall() == false && tower.bottomHasBall() == false && tower.topHasBall() == false) {
      this.state = State.NO_BALLS;
    } 
  }

  private void moveUnoBall() {
    if(tower.bottomHasBall() == false) {
      feeder.run(1);
      tower.run(1);
    }
  }

  private void moveUnoBallPart2() {
    if(feeder.hasBall() == false) {
      feeder.run(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
