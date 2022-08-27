// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.Utils;

public class DriveTrain extends SubsystemBase {
  private static Gyroscope gyro = Gyroscope.getInstance();

  private static DriveTrain instance;
  private final WPI_TalonFX motorLLead = new WPI_TalonFX(CANIds.MOTOR_L_LEAD_ID);
  private final WPI_TalonFX motorL1Follow = new WPI_TalonFX(CANIds.MOTOR_L1_FOLLOW_ID);
  private final WPI_TalonFX motorL2Follow = new WPI_TalonFX(CANIds.MOTOR_L2_FOLLOW_ID);
  private final WPI_TalonFX motorRLead = new WPI_TalonFX(CANIds.MOTOR_R_LEAD_ID);
  private final WPI_TalonFX motorR1Follow = new WPI_TalonFX(CANIds.MOTOR_R1_FOLLOW_ID);
  private final WPI_TalonFX motorR2Follow = new WPI_TalonFX(CANIds.MOTOR_R2_FOLLOW_ID);

  private SlewRateLimiter driveRLimiter = new SlewRateLimiter(1.5);
  private SlewRateLimiter driveLLimiter = new SlewRateLimiter(1.5);

  // private final DifferentialDrive diffDrive = new DifferentialDrive(motorLLead,
  // motorRLead);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      DriveTrainConstants.trackWidthMeters);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  private Pose2d pose;
  private Field2d field = new Field2d();

  public static DriveTrain getInstance() {
    if (instance == null) {
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

    motorLLead.setInverted(false);
    motorL1Follow.setInverted(InvertType.FollowMaster);
    motorL2Follow.setInverted(InvertType.FollowMaster);
    motorRLead.setInverted(true);
    motorR1Follow.setInverted(InvertType.FollowMaster);
    motorR2Follow.setInverted(InvertType.FollowMaster);

    motorLLead.setNeutralMode(NeutralMode.Coast);
    motorL1Follow.setNeutralMode(NeutralMode.Coast);
    motorL2Follow.setNeutralMode(NeutralMode.Coast);
    motorRLead.setNeutralMode(NeutralMode.Coast);
    motorR1Follow.setNeutralMode(NeutralMode.Coast);
    motorR2Follow.setNeutralMode(NeutralMode.Coast);

    SmartDashboard.putData("field", field);

    motorLLead.setSelectedSensorPosition(0);
    motorRLead.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(gyro.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters());
    field.setRobotPose(pose);
    SmartDashboard.putNumber("left encoder", getLeftEncoderMeters());
    SmartDashboard.putNumber("right encoder", getRightEncoderMeters());
    // SmartDashboard.putNumber("rot2Dangle", gyro.getRotation2d().getDegrees());
    // //uncomment if angle resetting breaks (we're trying to please the programming
    // gods) (its magic)
  }

  public void run(double speedRight, double speedLeft) {
    motorLLead.set(speedLeft);
    motorRLead.set(speedRight);
  }

  public void stop() {
    motorLLead.set(0);
    motorRLead.set(0);
  }

  public void tankDrive(double leftMotorPercentage, double rightMotorPercentage) {
    double driveLeft = Utils.bound(leftMotorPercentage, 1, -1);
    double driveRight = Utils.bound(rightMotorPercentage, 1, -1);
    motorLLead.set(driveLLimiter.calculate(driveLeft));
    motorRLead.set(driveRLimiter.calculate(driveRight));
  }

  public void arcadeDrive(double leftY, double leftX) {
    tankDrive(leftY + leftX, leftY - leftX);
  }

  public void setOdometry(Pose2d givenPose) {
    odometry.resetPosition(givenPose, gyro.getRotation2d());
  }

  public double getLeftEncoderMeters() {
    return motorLLead.getSelectedSensorPosition() * DriveTrainConstants.ticksToMeters;
  }

  public double getRightEncoderMeters() {
    return motorRLead.getSelectedSensorPosition() * DriveTrainConstants.ticksToMeters;
  }

  public double getLeftMetersPerSec() {
    return motorLLead.getSelectedSensorVelocity() * DriveTrainConstants.ticksToMeters;
  }

  public double getRightMetersPerSec() {
    return motorRLead.getSelectedSensorVelocity() * DriveTrainConstants.ticksToMeters;
  }

  /**
   * 
   */
  public double getAngleToHub() {
    double radians = Math.atan((FieldConstants.hubY - pose.getY()) / FieldConstants.hubX - pose.getX());

    //In the case that radians is undefined this method returns positive or negative 90
    //Enacted when the robot and hub share an x coordinate.
    if(pose.getX() == FieldConstants.hubX) {
      return 90.0 * Math.signum(FieldConstants.hubY - pose.getY());
    
    //Robot is to the left of the hub (Our alliance's side of the field)
    } else if(pose.getX() < FieldConstants.hubX) {
      return Math.toDegrees(radians);

    //Robot is to the right of the hub (Opposing alliance's side of the field)
    } else {
        return (180.0 - Math.abs(Math.toDegrees(radians))) * Math.signum(radians) * -1.0;
    }
  }

}
