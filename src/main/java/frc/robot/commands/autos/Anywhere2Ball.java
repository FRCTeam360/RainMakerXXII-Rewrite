// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConfig;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.QueueBalls;
import frc.robot.commands.RunFlywheelWithLimelight;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TurretAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Anywhere2Ball extends ParallelRaceGroup {

  Turret turret = Turret.getInstance();
  Limelight limelight = Limelight.getInstance();
  Intake intake = Intake.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();

  public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(),
    new Pose2d(0, 0, new Rotation2d(0)),
    AutoConfig.configFwdLow);

  /** Creates a new T_R_2ball. */
  public Anywhere2Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

        new TurretAuto(),

        new RunFlywheelWithLimelight(),

        new SequentialCommandGroup(

            // new AutoRunFeederAndTower(),

            new ExtendIntake(),

            new ParallelRaceGroup(

                new RunIntake(),

                new SequentialCommandGroup(

                    new AutoDrive(phase1),

                    new AutoShoot(2)
                )
            )
        )
    );
  }
}
