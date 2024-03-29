// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DigitalInputPorts;
import frc.robot.utils.CargoCounter;

public class Tower extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(CANIds.TOWER_ID, MotorType.kBrushless);
  private final DigitalInput topTower = new DigitalInput(DigitalInputPorts.TOP_TOWER);
  private final DigitalInput bottomTower = new DigitalInput(DigitalInputPorts.BOTTOM_TOWER);

  private BallTrackingState ballTrackingState = BallTrackingState.BOTTOM;
  private boolean pastBallInTop = false, pastBallInBottom = false;

  private CargoCounter mCargoCounter = CargoCounter.getInstance();

  private enum BallTrackingState {
    NO_BALL, BOTTOM, SHOOTING_ZONE
  }


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

  public boolean ballNotInBottom() {
    return bottomTower.get();
  }

  public boolean ballInBottom() {
    return !bottomTower.get();
  }

  public boolean ballInTop() {
    return !topTower.get();
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

  public void trackShots() {
    switch (this.ballTrackingState) {
      case NO_BALL:
        this.trackNoBall();
        break;
      case BOTTOM:
        this.trackBallInBottom();
        break;
      case SHOOTING_ZONE:
        this.trackBallInShootingZone();
        break;
      default:
    }
    this.setPastBallInBottom();
    this.setPastBallInTop();
  }

  public double getMotorSpeed() {
    return this.motor.getEncoder().getVelocity();
  }

  private boolean isRisingEdgeTop() {
    return ballInTop() && !this.pastBallInTop;
  }
  
  private boolean isFallingEdgeTop() {
    return !ballInTop() && this.pastBallInTop;
  }

  private boolean isRisingEdgeBottom() {
    return ballInBottom() && !this.pastBallInBottom;
  }
  
  private boolean isFallingEdgeBottom() {
    return !ballInBottom() && this.pastBallInBottom;
  }

  private void setPastBallInTop() {
    this.pastBallInTop = this.ballInTop();
  }

  private void setPastBallInBottom() {
    this.pastBallInBottom = this.ballInBottom();
  }

  private void trackNoBall(){
    if(ballInBottom()){
      this.ballTrackingState = BallTrackingState.BOTTOM;
    }
  }

  private void trackBallInBottom(){
    double motorSpeed = this.getMotorSpeed();
    boolean isFallingEdgeBottom = this.isFallingEdgeBottom();
    if(isFallingEdgeBottom && motorSpeed > 0){
      this.ballTrackingState = BallTrackingState.SHOOTING_ZONE;
    }else if(isFallingEdgeBottom && motorSpeed < 0){
      this.ballTrackingState = BallTrackingState.NO_BALL;
    }
  }

  private void trackBallInShootingZone(){
    double motorSpeed = this.getMotorSpeed();
    boolean isFallingEdgeTop = this.isFallingEdgeTop();
    if(isFallingEdgeTop && motorSpeed >= 0){
      this.ballTrackingState = BallTrackingState.NO_BALL;
      this.mCargoCounter.incrementShootCount();
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Tower Temp", tower.getMotorTemperature());
    SmartDashboard.putBoolean("Top Sensor", topTower.get());
    SmartDashboard.putBoolean("Bottom Sensor", bottomTower.get());
    SmartDashboard.putString("Tracking State", this.ballTrackingState.toString());
    // SmartDashboard.putNumber("Motor Speed", this.getMotorSpeed());

    this.trackShots();
  }
}
