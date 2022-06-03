// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.ExtendedXboxController;
import frc.robot.utils.OI;
import frc.robot.subsystems.DriveTrain;

public class FieldCentricDrive extends CommandBase {
  DriveTrain mDriveTrain = DriveTrain.getInstance();
  ExtendedXboxController driverCont = OI.driverCont;

  double rightLeft;
  double upDown;
  double forward;
  double right;
  double gyroAngle;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroAngle = Math.toRadians(mDriveTrain.getGyroAngle());
    rightLeft = driverCont.getLeftXSquared();
    upDown = driverCont.getLeftYSquared();
    forward = upDown * Math.cos(gyroAngle) + rightLeft * Math.sin(gyroAngle);
    right = -upDown * Math.sin(gyroAngle) + rightLeft * Math.cos(gyroAngle);
    mDriveTrain.arcadeDrive(forward, right);
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
