// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
  public final AHRS navX = new AHRS(SPI.Port.kMXP);
  private static Gyroscope instance;

  /** Creates a new Gyro. */
  private Gyroscope() {
    navX.reset();
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
    //System.out.println(navX.getAngle());
  }
  
  public double getGyroAngle() {
    return navX.getAngle();
  }

  public void setGyroAngle(double degrees) {
    navX.reset();
    navX.setAngleAdjustment(degrees);
  }

}
