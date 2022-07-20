// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DigitalInputPorts;

public class Feeder extends SubsystemBase {
  private static Feeder instance;
  private final CANSparkMax motor = new CANSparkMax(CANIds.FEEDER_ID, MotorType.kBrushless);
  private final DigitalInput feederSensor = new DigitalInput(DigitalInputPorts.FEEDER);

  /** Creates a new Feeder. */
  public Feeder() {}

  public static Feeder getInstance(){
    if(instance == null){
      instance = new Feeder();
    }
    return instance;
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  public boolean hasBall() {
    return !feederSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
