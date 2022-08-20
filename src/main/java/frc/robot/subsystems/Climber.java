// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;
import frc.robot.utils.OI;

public class Climber extends SubsystemBase {
  private static Climber instance;
  private final CANSparkMax left = new CANSparkMax(CANIds.CLIMBER_L_ID, MotorType.kBrushless);
  private final CANSparkMax right = new CANSparkMax(CANIds.CLIMBER_R_ID, MotorType.kBrushless); 

  private final SparkMaxPIDController leftPIDController = left.getPIDController();
  private final SparkMaxPIDController rightPIDController = right.getPIDController();
  private final RelativeEncoder leftEncoder = left.getEncoder();
  private final RelativeEncoder righEncoder = right.getEncoder();
  
  private double kP = 0;

  private double difference;

  private final XboxController operatorCont = OI.operatorCont;
  /** Creates a new Climber. */
  public Climber() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);

    left.setInverted(false);
    right.setInverted(true);

    leftPIDController.setP(kP);
    rightPIDController.setP(kP);
  }

  public static Climber getInstance(){
    if(instance == null){
      instance = new Climber();
    }
    return instance;
  }

  private double getLeftPosition(){
    return leftEncoder.getPosition();
  }

  private double getRightPosition(){
    return righEncoder.getPosition();
  }

  public void runLeft(double speed){
    left.set(speed);
  }

  public void runRight(double speed){
    right.set(speed);
  }

  public void extendClimber(){
    difference = getLeftPosition() - getRightPosition();
    double diff = difference / 100;
    runLeft(0.8 - diff);
    runRight(0.8 + diff);
  }

  public void retractClimber(){
    difference = Math.abs(getLeftPosition() - getRightPosition());
    double diff = difference / 100;
    runLeft(-0.8 - diff);
    runRight(-0.8 + diff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
