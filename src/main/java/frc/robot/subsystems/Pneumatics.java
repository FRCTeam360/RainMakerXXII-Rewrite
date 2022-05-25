// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
public class Pneumatics extends SubsystemBase{

  private Compressor compressor = new Compressor(CANIds.PNEUMATIC_ID, PneumaticsModuleType.CTREPCM);
  private static Pneumatics instance;

  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  public static Pneumatics getInstance(){
    if(instance == null){
      instance = new Pneumatics();
    }

    return instance;
  }

  /**
   * Starts compressor
   */
  public void enable(){
    System.out.println("Enable pneumatics"); //test later
    //To do figure out why it's pressurizing on its own.
    compressor.enableDigital();
  }

  /**
   * Stops compressor
   */
  public void disable(){
    compressor.disable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
