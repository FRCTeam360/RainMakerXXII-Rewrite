// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.Timer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoConfig;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TR_5Ball extends ParallelRaceGroup {

  Turret turret = Turret.getInstance();
  Limelight limelight = Limelight.getInstance();
  Intake intake = Intake.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();
  Gyroscope gyro = Gyroscope.getInstance();

  // TO DO: UPDATE WHEN ODOMETRY IS FIXED !!! (names indicate starts and ends of
  // each path)
  private static final String ball2JSON = "paths/2ball.wpilib.json";

  public static final Trajectory initToBall2 = TrajectoryGenerator.generateTrajectory( // robot starts w ball 1
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.1, -1, new Rotation2d(-90)),
      // TODO create AutoConfig
      AutoConfig.configFwdHigh);

  public static final Trajectory ball2ToBall3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.1, -1, new Rotation2d(-90)),
      List.of(
          new Translation2d(.25, -3)),
      new Pose2d(0, -3.5, new Rotation2d(-90)),
      AutoConfig.configFwdHigh);

  public static final Trajectory ball3ToBall4 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, -3.5, new Rotation2d(-90)),
      List.of(
      // new Translation2d(0, -7)),
      ),
      new Pose2d(0.60 + AutoConstants.getXOffsetTerminal5Ball(), -7 + AutoConstants.getYOffsetTerminal5Ball(),
          new Rotation2d(-45)),
      AutoConfig.configFwdHigh);

  private static final Trajectory ball4ToBall5 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0.60 + AutoConstants.getXOffsetTerminal5Ball(), -7 + AutoConstants.getYOffsetTerminal5Ball(),
          new Rotation2d(-45)),
      List.of(),
      new Pose2d(0.5 + AutoConstants.getXOffsetTerminal5Ball(), -6.9 + AutoConstants.getYOffsetTerminal5Ball(),
          new Rotation2d(-45)),
      AutoConfig.configRevHigh);

  private static final Trajectory ball5ToHub = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0.5 + AutoConstants.getXOffsetTerminal5Ball(), -6.9 + AutoConstants.getYOffsetTerminal5Ball(),
          new Rotation2d(-45)),
      List.of(),
      new Pose2d(0, -3.5, new Rotation2d(-45)),
      AutoConfig.configRevHigh);

  /** Creates a new T_R_2ball. */
  public TR_5Ball() {
    addCommands(
        new TurretAuto(),
        new RunFlywheelWithLimelight(),

        new SequentialCommandGroup(
          // new InstantCommand(() -> gyro.setGyroAngle(1.5)),
          new ExtendIntake(),
          
          new ParallelRaceGroup(
            new RunIntake(),
            new AutoDrive(initToBall2).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new QueueBalls(true)
          ),
          new RetractIntake(),
          new ParallelRaceGroup(
            new RunIntake(),
            new AutoShoot(2)
          ),
          new ExtendIntake(),
          new ParallelRaceGroup(
            new RunIntake(),
            new AutoDrive(ball2ToBall3).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new QueueBalls(true)
          ),
          new ParallelRaceGroup(
            new SequentialCommandGroup(
              new RetractIntake(),
              new RunIntake()
            ),
            new QueueBalls(true),
            new SequentialCommandGroup(
              new AutoDrive(ball3ToBall4).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
              new AutoDrive(ball4ToBall5).andThen(() -> driveTrain.tankDriveVolts(0,0)),
              new WaitCommand(.5),
              new AutoDrive(ball5ToHub).andThen(() -> driveTrain.tankDriveVolts(0, 0))
            )
            ),
            new InstantCommand(() -> gyro.setAngleAdjustment(1.5)),
            new ParallelCommandGroup(
              new RetractIntake(),
              new AutoShoot(2)
            )
        )
    ); 
  }

}
