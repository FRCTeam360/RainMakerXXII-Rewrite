// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.utils.OI;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Turret turret = Turret.getInstance();
  private final DriveTrain driveTrain = DriveTrain.getInstance();

  private final RunIntake runIntake = new RunIntake();
  private final RunIntake runIntakeReverse = new RunIntake(true);
  private final RunFeeder runFeeder = new RunFeeder();
  private final RunFeeder runFeederReversed = new RunFeeder(true);
  private final RunTower runTower = new RunTower();
  private final RunTower runTowerReversed = new RunTower(true);
  private final ExtendIntake extendIntake = new ExtendIntake();
  private final RetractIntake retractIntake = new RetractIntake();
  private final TankDrive tankDrive = new TankDrive();
  private final ArcadeDrive arcadeDrive = new ArcadeDrive();
  private final FieldOrientedDrive fieldDrive = new FieldOrientedDrive();
  private final SetGyroAngle resetGyroAngle = new SetGyroAngle(0.0);
  private final QueueBalls queueBalls = new QueueBalls(false);
  private final QueueBalls shootBalls = new QueueBalls(true);
  private final ClimbManual climb = new ClimbManual();
  private final TurretManual turretManual = new TurretManual();
  private final TurretAuto turretAuto = new TurretAuto();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    OI.runIntakeButton.whileHeld(runIntake);
    OI.extendIntakeButton.whileHeld(extendIntake);
    OI.retractIntakeButton.whileHeld(retractIntake);
    OI.runIntakeButton.whileHeld(runIntake);
    OI.runIntakeButton.and(OI.reverseButton).whileActiveContinuous(runIntakeReverse);
    OI.runFeederButton.whileHeld(runFeeder);
    OI.runFeederButton.and(OI.reverseButton).whileActiveContinuous(runFeederReversed);
    OI.runTowerButton.whileHeld(runTower);
    OI.runTowerButton.and(OI.reverseButton).whileActiveContinuous(runTowerReversed);
    OI.queueBallsButton.whileHeld(queueBalls);

    OI.tankButton.whenPressed(tankDrive);
    OI.arcadeButton.whenPressed(arcadeDrive);
    OI.fieldOrientedButton.whenPressed(fieldDrive);
    OI.resetButton.whenPressed(resetGyroAngle);

    OI.climbButton.whileHeld(climb);
    OI.manualTurretButton.whileHeld(turretManual);
  }

  private void configureDefaultCommands() {
    turret.setDefaultCommand(turretAuto);
    driveTrain.setDefaultCommand(fieldDrive);
    // turret.setDefaultCommand(turretManual);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

