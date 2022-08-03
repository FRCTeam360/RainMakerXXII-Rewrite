// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static Limelight instance;
  private NetworkTable limelightData = NetworkTableInstance.getDefault().getTable("limelight");
  /** Creates a new Limelight. */
  public Limelight() {}

  public static Limelight getInstance() {
    if(instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  /**
   * Gets the horizontal offset from center of vision to target in degrees
   * @return horizontal offset (degrees)
  */
  public double getTX() {
    return limelightData.getEntry("tX").getDouble(0);
  }

  /**
   * Gets vertical offset from center of vision to target in degrees
   * @return vertical offset (degrees)
   */
  public double getTY() {
    return limelightData.getEntry("tY").getDouble(0);
  }

  /**
   * Checks for prescence of valid target
   * @return 1 for valid target, 0 for no valid target
   */
  public double getTV() {
    return limelightData.getEntry("tV").getDouble(0);
  }

  public boolean hasValidTarget() {
    return getTV() == 1;
  }

  public boolean isOnTarget() {
    return Math.abs(getTX()) <= 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
