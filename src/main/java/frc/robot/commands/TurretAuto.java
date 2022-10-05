// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TurretAuto extends CommandBase {
  private Turret turret = Turret.getInstance();
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private Limelight limelight = Limelight.getInstance();
  /** Creates a new TurretAuto. */
  public TurretAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("autoing");
    if(limelight.hasValidTarget()) {
      System.out.println("autoing");
      turret.alignWithLime();
    } else {
      turret.run(0);
      // turret.turnToFieldAngle(driveTrain.getAngleToHub());
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
