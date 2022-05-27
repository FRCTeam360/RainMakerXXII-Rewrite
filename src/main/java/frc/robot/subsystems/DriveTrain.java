// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;

public class DriveTrain extends SubsystemBase {
  private static DriveTrain instance;
  private final WPI_TalonFX motorLLead = new WPI_TalonFX(CANIds.MOTOR_L_LEAD_ID);
  private final WPI_TalonFX motorL1Follow = new WPI_TalonFX(CANIds.MOTOR_L1_FOLLOW_ID);
  private final WPI_TalonFX motorL2Follow = new WPI_TalonFX(CANIds.MOTOR_L2_FOLLOW_ID);
  private final WPI_TalonFX motorRLead = new WPI_TalonFX(CANIds.MOTOR_R_LEAD_ID);
  private final WPI_TalonFX motorR1Follow = new WPI_TalonFX(CANIds.MOTOR_R1_FOLLOW_ID);
  private final WPI_TalonFX motorR2Follow = new WPI_TalonFX(CANIds.MOTOR_R2_FOLLOW_ID);

  private final DifferentialDrive diffDrive = new DifferentialDrive(motorLLead, motorRLead);

  public static DriveTrain getInstance() {
    if(instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    motorL1Follow.follow(motorLLead);
    motorL2Follow.follow(motorLLead);
    motorR1Follow.follow(motorRLead);
    motorR2Follow.follow(motorRLead);
  }

  public void run(double speedRight, double speedLeft) {
    motorLLead.set(speedLeft);
    motorRLead.set(speedRight);
  }

  public void stop() {
    motorLLead.set(0);
    motorRLead.set(0);
  }

  public void tankDrive(double leftY, double rightY) {
    diffDrive.tankDrive(leftY, rightY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
