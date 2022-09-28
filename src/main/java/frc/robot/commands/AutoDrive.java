// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class AutoDrive extends RamseteCommand {
  private final DriveTrain driveTrain = DriveTrain.getInstance();
  
  public AutoDrive (Trajectory traj, DriveTrain driveTrain) {
    super(
        traj, 
        driveTrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        driveTrain.feedForward,
        driveTrain.getKinematics(),
        driveTrain::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        driveTrain::tankDriveVolts,
        driveTrain
    ); 
}
}
