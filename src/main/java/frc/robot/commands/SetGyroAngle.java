// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyroscope;

public class SetGyroAngle extends CommandBase {
  private double angle;
  private Gyroscope gyroscope = Gyroscope.getInstance();

  /** Creates a new SetGyroAngle. */
  public SetGyroAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroscope.setGyroAngle(angle);
    System.out.println("in SetGyroAngle");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
