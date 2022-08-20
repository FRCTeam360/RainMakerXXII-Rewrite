// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Turret extends SubsystemBase {
  private static Turret instance;

  private final CANSparkMax motor = new CANSparkMax(CANIds.TURRET_ID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkMaxPIDController pidController;
  
  public static final double gearBoxRatio = 1.0 / 20.0;
  public static final double pulleyRatio = 1.5 / 17.5;
  public static final double degreesPerRotation = 360.0 / 1.0;
  public static final double rotationsPerTick = 1.0 / 42.0;
  public static final double conversionFactor = gearBoxRatio * pulleyRatio * degreesPerRotation;

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  
  /** Creates a new Turret. */
  private Turret() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    motor.setInverted(false);

    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(conversionFactor);
    // encoder.setPositionConversionFactor(0);

    pidController = motor.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    resetAngle(0);
  }

  public static Turret getInstance(){
    if(instance == null){
      instance = new Turret();
    }
    return instance;
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  public void resetAngle(double angle) {
    encoder.setPosition(angle);
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void turnToAngle(double angle) {
    pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret angle", getAngle());
    System.out.println("turret angle is " + getAngle());
  }
}
