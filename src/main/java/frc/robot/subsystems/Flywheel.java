// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;

public class Flywheel extends SubsystemBase {
  private static Flywheel instance;
  private final TalonFX motorLead = new TalonFX(CANIds.FLYWHEEL_LEAD_ID);
  private final TalonFX motorFollow = new TalonFX(CANIds.FLYWHEEL_FOLLOW_ID);
  private double targetVelocity;
  /** Creates a new Flywheel. */
  public Flywheel() {
    motorLead.configFactoryDefault();
    motorFollow.configFactoryDefault();

    motorLead.setNeutralMode(NeutralMode.Coast);
    motorFollow.setNeutralMode(NeutralMode.Coast);

    motorLead.setInverted(false);
    motorFollow.setInverted(true);
  }

  public static Flywheel getInstance() {
    if(instance == null) {
      instance = new Flywheel();
    }
    return instance;
  }

  public void setPower(double power) {
    motorLead.set(ControlMode.PercentOutput, power);
  }

  public void setVelocity(double velocity) {
    motorLead.set(ControlMode.Velocity, velocity);
    targetVelocity = velocity;
  }

  public double getCurrentVelocity() {
    return motorLead.getSelectedSensorVelocity();
  }

  public boolean isAtSpeed() {
    return Math.abs(getCurrentVelocity() - targetVelocity) <= 50;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}