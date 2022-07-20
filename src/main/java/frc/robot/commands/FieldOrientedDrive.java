// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.ExtendedXboxController;
import frc.robot.utils.OI;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;

public class FieldOrientedDrive extends CommandBase {

  DriveTrain driveTrain = DriveTrain.getInstance();
  ExtendedXboxController driverCont = OI.driverCont;
  Gyroscope gyroscope = Gyroscope.getInstance();

  double rightLeft;
  double upDown;
  double forward;
  double right;
  double gyroAngle;

  public FieldOrientedDrive() {
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroAngle = Math.toRadians(gyroscope.getGyroAngle());
    System.out.println(gyroAngle);
    rightLeft = -driverCont.getLeftXSquared();
    upDown = driverCont.getLeftYSquared();
    forward = upDown * Math.cos(gyroAngle) + rightLeft * Math.sin(gyroAngle);
    right = -upDown * Math.sin(gyroAngle) + rightLeft * Math.cos(gyroAngle);

    Double turnInPlace = driverCont.getRightX();
    if(OI.FODReverseButton.get()) {
      driveTrain.arcadeDrive(forward, -right - turnInPlace);
    } else {
      driveTrain.arcadeDrive(forward, right - turnInPlace);
    }
    
    System.out.println("fielding");

  }
  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}