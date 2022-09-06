// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DigitalInputPorts;

public class Tower extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(CANIds.TOWER_ID, MotorType.kBrushless);
  private final DigitalInput topTower = new DigitalInput(DigitalInputPorts.TOP_TOWER);
  private final DigitalInput bottomTower = new DigitalInput(DigitalInputPorts.BOTTOM_TOWER);


  private static Tower instance;
  /** Creates a new Tower. */
  public Tower() {
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
  }

  public static Tower getInstance(){
    if(instance == null){
      instance = new Tower();
    }
    return instance;
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  /**
   * checks sensor in top of tower
   * @return
   */
  public boolean topHasBall() {
    return !topTower.get();
  }

  /**
   * checks sensor in bottom of tower
   * @return
   */
  public boolean bottomHasBall() {
    return !bottomTower.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
