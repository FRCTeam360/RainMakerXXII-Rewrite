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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    left.getEncoder().setPosition(0);
    right.getEncoder().setPosition(0);

    left.setSoftLimit(SoftLimitDirection.kForward, 50);
    left.setSoftLimit(SoftLimitDirection.kReverse, 0);
    right.setSoftLimit(SoftLimitDirection.kForward, 50);
    right.setSoftLimit(SoftLimitDirection.kReverse, 0);

    left.enableSoftLimit(SoftLimitDirection.kForward, true);
    left.enableSoftLimit(SoftLimitDirection.kReverse, false);
    right.enableSoftLimit(SoftLimitDirection.kForward, true);
    right.enableSoftLimit(SoftLimitDirection.kReverse, false);
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

  //code finds the difference in encoder position then adjusts the output to make one catch up to the other

  public void extendClimber(){
    difference = getLeftPosition() - getRightPosition();
    // /100 is arbitrary, we could use a different value
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
    SmartDashboard.putNumber("LC encoder", left.getEncoder().getPosition());
    SmartDashboard.putNumber("RC encoder", right.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }
}
