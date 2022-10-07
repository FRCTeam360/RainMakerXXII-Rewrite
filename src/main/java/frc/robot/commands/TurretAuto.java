// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TurretAuto extends CommandBase {
  private Turret turret = Turret.getInstance();
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private Limelight limelight = Limelight.getInstance();

  private boolean limitHasBeenHit;

  public enum Mode {
    LIMETIME, SEEK_LEFT, SEEK_RIGHT
  };

  private Mode mode = Mode.LIMETIME;

  private Timer timer = new Timer();

  /** Creates a new TurretAuto. */
  public TurretAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // // System.out.println("autoing");
    // if(limelight.hasValidTarget()) {
    // System.out.println("autoing");
    // turret.alignWithLime();
    // timer.stop();
    // timer.reset();
    // limitHasBeenHit = false;
    // } else if(timer.get() >= 1) {
    // turret.search(limitHasBeenHit);
    // // turret.turnToFieldAngle(driveTrain.getAngleToHub());
    // }

    switch (mode) {
      default:
      case LIMETIME:
        limetime();
        break;
      case SEEK_LEFT:
        turret.run(0.8);
        if (limelight.hasValidTarget()) {
          mode = Mode.LIMETIME;
        } else if (turret.getAngle() >= Turret.leftSoftLimit) {
          mode = Mode.SEEK_RIGHT;
        }
        break;
      case SEEK_RIGHT:
        turret.run(-0.8);
        if (limelight.hasValidTarget()) {
          mode = Mode.LIMETIME;
        } else if (turret.getAngle() <= Turret.rightSoftLimit) {
          mode = Mode.SEEK_LEFT;
        }
    }
  }

  public void limetime() {
    turret.alignWithLime();
        if (!limelight.hasValidTarget()) {
          timer.start();
          if (turret.getAngle() < 0 && timer.get() > 1) {
            mode = Mode.SEEK_LEFT;
          } else if(timer.get() > 1) {
            mode = Mode.SEEK_RIGHT;
          }
        } else {
          timer.stop();
          timer.reset();
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
