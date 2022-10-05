// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DigitalInputPorts;

public class Turret extends SubsystemBase {
  private static Turret instance;

  private Gyroscope gyro = Gyroscope.getInstance();
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private Limelight limelight = Limelight.getInstance();

  private final CANSparkMax motor = new CANSparkMax(CANIds.TURRET_ID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkMaxPIDController pidController;
  
  //ensure you have .0 to make these of type double rather than int
  public static final double gearBoxRatio = 1.0 / 20.0;
  public static final double pulleyRatio = 1.5 / 17.5;
  public static final double degreesPerRotation = 360.0 / 1.0;
  public static final double rotationsPerTick = 1.0 / 42.0;
  public static final double conversionFactor = gearBoxRatio * pulleyRatio * degreesPerRotation;

  public static final float leftSoftLimit = 170;
  public static final float rightSoftLimit = -170;
  public static final float leftSoftLimitEncoder = (float) (leftSoftLimit / conversionFactor);
  public static final float rightSoftLimitEncoder = (float) (rightSoftLimit / conversionFactor);

  private DigitalInput limitSwitch = new DigitalInput(DigitalInputPorts.LIMIT_SWITCH);
  private boolean pastLimitSwitchState;

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  public static final double kPLimelight = 0.030; // values may be altered, seperate for clarification , changer
  // kPLimelight from .05
  public static final double kILimelight = 0; // *
  public static final double kDLimelight = 0.04; // * changed from 0.01
  public static final double kFLimelight = 0; // *

  private double alignIntegral;
  private double previousTX;

  
  /** Creates a new Turret. */
  private Turret() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);

    encoder = motor.getEncoder();
    pidController = motor.getPIDController();

    //changes output from motor ticks to degrees
    encoder.setPositionConversionFactor(conversionFactor);
    // encoder.setPositionConversionFactor(0);

    pidController = motor.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    motor.setSoftLimit(SoftLimitDirection.kForward, leftSoftLimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse, rightSoftLimit);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    resetAngle(0);
  }

  public static Turret getInstance(){
    if(instance == null){
      instance = new Turret();
    }
    return instance;
  }

  /**
   * uses encoder to get position of the turret
   * @return angle of turret in degrees
   */
  public double getAngle() {
    return encoder.getPosition();
  }

  /**
   * sets turret encoder to a position
   * use 0 to reset
   * @param angle in degrees
   */
  public void resetAngle(double angle) {
    encoder.setPosition(angle);
  }

  /**
   * sets turret motor
   * @param speed percent speed from -1.0 to 1.0
   */
  public void run(double speed) {
    motor.set(speed);
  }

  /**
   * uses position pid to turn the turret to set robot relative angle
   * @param angle in degrees
   */
  public void turnToTurretAngle(double angle) {
    pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  //Turns turret relative to robot heading
  public void turnToRobotAngle(double angle) {
    turnToTurretAngle(angle - (180 * Math.signum(angle)));
  }

  //Turns turret relative to field
  public void turnToFieldAngle(double angle) {
    turnToRobotAngle(angle - gyro.getGyroAngle());
  }

  public void alignWithLime() {

    double currentTX = limelight.getTX();

    double aimError = - shootOnMove(currentTX);
    double deriv = aimError - previousTX;

    alignIntegral = alignIntegral + aimError;
    previousTX = aimError;

    double aimAdjust = (aimError * Turret.kPLimelight) + (alignIntegral * Turret.kILimelight)
        - (deriv * Turret.kDLimelight) + (kFLimelight);

    this.run(aimAdjust);
  }

  public double getTargetRelativeVelocity(){
    return driveTrain.getAverageMetersPerSec() * Math.sin(Math.toRadians(getAngle()));
  }

  private double shootOnMove(double angle){
    return angle + (getTargetRelativeVelocity() * 125);
  }

  public boolean checkLimitSwitch() {
    return limitSwitch.get();
  }
  
  public void limitSwitchResetAngle() {
    boolean currentMiddleLimitState = this.checkLimitSwitch();

    if (currentMiddleLimitState == false && pastLimitSwitchState == true) {
        resetAngle(0);
        System.out.println("lil zero");
    }

    pastLimitSwitchState = currentMiddleLimitState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret angle", getAngle());
    System.out.println("turret angle is " + getAngle());

    limitSwitchResetAngle();
  }
}
