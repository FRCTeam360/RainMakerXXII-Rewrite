// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static Limelight instance;
  private NetworkTable limelightData = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = limelightData.getEntry("tv");
  private NetworkTableEntry tx = limelightData.getEntry("tx");
  private NetworkTableEntry ty = limelightData.getEntry("ty");
  private NetworkTableEntry ta = limelightData.getEntry("ta");
  private NetworkTableEntry tl = limelightData.getEntry("tl");
  private NetworkTableEntry snap = limelightData.getEntry("snapshot");
  private NetworkTableEntry botPose = limelightData.getEntry("botpose");
  
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
    return tx.getDouble(0.0);
  }

  /**
   * Gets vertical offset from center of vision to target in degrees
   * @return vertical offset (degrees)
   */
  public double getTY() {
    return ty.getDouble(0.0);
  }

  /**
   * Checks for prescence of valid target
   * @return 1 for valid target, 0 for no valid target
   */
  public double getTV() {
    // return limelightData.getEntry("tV").getDouble(0);
    return tv.getDouble(0.0);
  }

  public boolean hasValidTarget() {
    return getTV() == 1;
  }

  public boolean isOnTarget() {
    return Math.abs(getTX()) <= 1 && hasValidTarget();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isOnTarget", isOnTarget());
    SmartDashboard.putBoolean("valid target", hasValidTarget());
    double[] doubleArray = new double[6];
    doubleArray = botPose.getDoubleArray(doubleArray);

    
    System.out.println("bot pose 1 :" + doubleArray[0]);
    System.out.println("bot pose 2 :" + doubleArray[1]);
    System.out.println("bot pose 3 :" + doubleArray[2]);
    System.out.println("bot pose 4 :" + doubleArray[3]);
    System.out.println("bot pose 5 :" + doubleArray[4]);
    System.out.println("bot pose 6 :" + doubleArray[5]);
    // This method will be called once per scheduler run
  }
}
