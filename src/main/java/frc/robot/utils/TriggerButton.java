// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/** Add your docs here. */
public class TriggerButton extends Button {
  private final XboxController mCont;
  public enum TriggerSide {
    LEFT, RIGHT
  }
  private TriggerSide mSide;

  /**
   * Creates a trigger for triggering commands.
   * 
   * @param cont The XboxController object that has the trigger
   * @param side The side of the controller that has the trigger
   */
  public TriggerButton(XboxController cont, TriggerSide side){
    requireNonNullParam(cont, "cont", "TriggerButton");

    mCont = cont;
    mSide = side;
  }
  
  /**
   * Gets the value of the trigger button
   * 
   * @return The value of the trigger button
   */
  @Override
  public boolean get() {
    if(mSide == TriggerSide.RIGHT){
      return mCont.getRightTriggerAxis() >= .9;
    }
    return mCont.getLeftTriggerAxis() >= .9;
  }
}
