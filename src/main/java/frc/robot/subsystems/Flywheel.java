// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIds;

public class Flywheel extends SubsystemBase {
  private static Flywheel instance;
  private final WPI_TalonFX motorLead = new WPI_TalonFX(CANIds.FLYWHEEL_LEAD_ID);
  private final WPI_TalonFX motorFollow = new WPI_TalonFX(CANIds.FLYWHEEL_FOLLOW_ID);

  private final Limelight limelight = Limelight.getInstance();

  private static final double a = 0.0086455746;
  private static final double b = -0.1318250336;
  private static final double c = 0.7137822277;
  private static final double d = -30.47373396;
  private static final double e = 3327.558816;

  // Old data, need to tune
  public static final int kSlotIdx = 0;
  public static final int kTimeOutMs = 30;
  public static final int kPIDLoopIdx = 0;
  public static double kP = 0.3;
  public static double kI = 0.000083;
  public static double kD = 0;
  public static double kF = 0.0471;
  public static double kIz = 200;
  public static final double kPeakOutput = 1;

  public static final double MAX_SHOOTER_ACCELERATION = 5000;
  private final SlewRateLimiter filter = new SlewRateLimiter(MAX_SHOOTER_ACCELERATION);

  public static final double shooterToRPM = (600.0 / 2048.0) * (3.0 / 2.0);
  public static final double shooterToMotorRPM = (600.0 / 2048.0);

  private double targetVelocity = 2000;
  /** Creates a new Flywheel. */
  public Flywheel() {
    motorLead.configFactoryDefault();
    motorFollow.configFactoryDefault();

    motorLead.setNeutralMode(NeutralMode.Coast);
    motorFollow.setNeutralMode(NeutralMode.Coast);

    motorLead.setInverted(false);
    motorFollow.setInverted(true);

    motorFollow.follow(motorLead);

    motorLead.config_kF(0, kF, kTimeOutMs);
    motorLead.config_kP(0, kP, kTimeOutMs);
    motorLead.config_kI(0, kI, kTimeOutMs);
    motorLead.config_kD(0, kD, kTimeOutMs);
    motorLead.config_IntegralZone(0, kIz, kTimeOutMs);
    motorLead.configNominalOutputForward(0, kTimeOutMs);
    motorLead.configNominalOutputReverse(0, kTimeOutMs);
    motorLead.configPeakOutputForward(1, kTimeOutMs);
    motorLead.configPeakOutputReverse(-1, kTimeOutMs);
  }

  public static Flywheel getInstance() {
    if(instance == null) {
      instance = new Flywheel();
    }
    return instance;
  }

  /**
   * sets shooter to a percent power
   * @param power percent power from -1.0 to 1.0
   */
  public void setPower(double power) {
    motorLead.set(power);
  }

  /**
   * sets shooter to velocity using pid controller
   * @param velocity in rpms maybe? honestly not sure
   */
  public void setVelocity(double velocity) {
    SmartDashboard.putNumber("set velocity", velocity);
    SmartDashboard.putNumber("Shoot Goal", targetVelocity);
    if (velocity == 0) {
      targetVelocity = 0;
    } else {

      // target = shootOnMove(target);
      
      targetVelocity = filter.calculate(velocity);


      motorLead.set(ControlMode.Velocity, targetVelocity * 2 / 3 / shooterToMotorRPM);
    }
  }

  private void stopIfShouldStop(){
    if(targetVelocity == 0){
      filter.reset(0);
      this.setPower(0);
    }
  }

  public double getCurrentVelocity() {
    return motorLead.getSelectedSensorVelocity() * shooterToRPM;
  }

  //50 is arbitrary, tune to account for issues
  public boolean isAtSpeed() {
    if(targetVelocity == 0){
      return false;
    }
    return Math.abs(getCurrentVelocity() - targetVelocity) <= 50;
  }

   /**
   * gets shoot goal as determined by our quartic regression, using limelight
   * y-value
   * 
   * @return shootGoal
   */
  public double getShootGoal() {
    double limedY = limelight.getTY();
    return ((a * Math.pow(limedY, 4)) + (b * Math.pow(limedY, 3) + (c * Math.pow(limedY, 2)) + (d * limedY) + e)) * 1.00;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", getCurrentVelocity());
    stopIfShouldStop();
    // This method will be called once per scheduler run
  }
}
