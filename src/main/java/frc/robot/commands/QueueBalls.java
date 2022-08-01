// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Tower;
import frc.robot.utils.BooleanProvider;


public class QueueBalls extends CommandBase {
  Feeder feeder = Feeder.getInstance();
  Tower tower = Tower.getInstance();
  Flywheel flywheel = Flywheel.getInstance();
  Limelight limelight = Limelight.getInstance();
  private enum State{NO_BALLS, ONE_BALL, ONE_PLUS_ONE_BALL, PRIMED, TWO_BALLS, FULL, JAM, SHOOT};
  private State state;
  private boolean shootWhenReady;
  
  
  /** Creates a new QueueBalls. */
  public QueueBalls(boolean shouldShootWhenReady) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder, tower);
    shootWhenReady = shouldShootWhenReady;
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
    if(shootWhenReady && flywheel.isAtSpeed() && limelight.hasValidTarget()) {
      state = State.SHOOT;
    }

    //Logic should work its way down the switch statement, moving from one state to the next as each step
    //is completed. In case of unknown state (null) updateState() is called. 
    switch(state) {
      default:
        stopFeederAndTower();
        updateState();
        break;
      case NO_BALLS:
        runFeederAndTower();
        if(tower.bottomHasBall()) {
          state = State.ONE_BALL;
        }
        break;
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
        pullBallUp();
        if(tower.topHasBall() && tower.bottomHasBall()) {
          state = State.FULL;
        } else if(tower.topHasBall() && !tower.bottomHasBall()) {
          state = State.JAM;
        }
        break;
      case FULL:
        stopFeederAndTower();
        break;
      case JAM:
        tower.run(-0.1);
        feeder.run(-0.1);
        if(tower.bottomHasBall()) {
          state = null;
        }
        break;
      case SHOOT:
        runFeederAndTower();
        break;
    }
  }

  //This logic will determine where in the switch statement the balls are in case of unknown (null) state
  private void updateState() {
    if (!tower.bottomHasBall() && !tower.topHasBall()) {
      state = State.NO_BALLS;
    } else if (tower.bottomHasBall() && !tower.topHasBall() && feeder.hasBall()) {
      state = State.ONE_PLUS_ONE_BALL;
    } else if (tower.bottomHasBall() && !tower.topHasBall() && !feeder.hasBall()) {
      state = State.ONE_BALL;
    } else if (tower.bottomHasBall() && tower.topHasBall()) {
      state = State.TWO_BALLS;
    } else if (!tower.bottomHasBall() && tower.topHasBall()) {
      state = State.JAM;
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

  private void pullBallUp() {
    tower.run(0.1);
    feeder.run(0.5);
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
