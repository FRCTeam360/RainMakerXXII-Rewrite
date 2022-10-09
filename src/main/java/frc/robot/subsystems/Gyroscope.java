// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.Constants.CANIds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
  public final AHRS navX = new AHRS(SPI.Port.kMXP);
  // public final WPI_Pigeon2 pigeon = new WPI_Pigeon2(CANIds.PIGEON_ID);
  private static Gyroscope instance;

  /** Creates a new Gyro. */
  private Gyroscope() {
    navX.reset();
    // pigeon.reset();
  }

  public static Gyroscope getInstance() {
    if (instance == null) {
      instance = new Gyroscope();
    }

    return instance;
  }

  @Override
  public void periodic() {
    // This method Fwill be called once per scheduler run
    // System.out.println(navX.getAngle());
    SmartDashboard.putNumber("angle", getGyroAngle());
  }
  
  public double getGyroAngle() {
    // return Math.IEEEremainder(pigeon.getAngle(), 360); 
    // return pigeon.getAngle();
    // return Math.IEEEremainder(navX.getAngle(), 360);
    return navX.getAngle();
  }

  public Rotation2d getRotation2d(){
    // return pigeon.getRotation2d();
    return navX.getRotation2d();
  }

  public void setGyroAngle(double degrees) {
    navX.reset();
    navX.setAngleAdjustment(degrees);
    // pigeon.reset();
    // pigeon.setYaw(degrees);
  }

  public void setAngleAdjustment(double degrees){
    navX.setAngleAdjustment(degrees + 90);
  }

}
