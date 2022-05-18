// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticChannels;

public class IntakePneumatics extends SubsystemBase {

  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticChannels.INTAKE_FORWARD, PneumaticChannels.INTAKE_REVERSE);

  private static IntakePneumatics instance;

  /** Creates a new IntakePneumatics. */
  public IntakePneumatics() {}

  public IntakePneumatics getInstance(){
    if (instance == null) {
      instance = new IntakePneumatics();
    }
    return instance;
  }

  public void extend(){
    solenoid.set(Value.kForward);
  }

  public void retract(){
    solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
