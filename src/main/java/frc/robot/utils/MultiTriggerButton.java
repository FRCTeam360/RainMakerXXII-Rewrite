// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.utils.TriggerButton.TriggerSide;

/** Add your docs here. */
public class MultiTriggerButton extends Button {
  private final XboxController mCont1;
  private final XboxController mCont2;

  private TriggerSide mSide;

  /**
   * Creates a trigger for triggering commands.
   * 
   * @param cont The XboxController object that has the trigger
   * @param left The side of the controller that has the trigger
   */
  public MultiTriggerButton(XboxController cont1, XboxController cont2, TriggerSide side){
    requireNonNullParam(cont1, "cont1", "TriggerButton");
    requireNonNullParam(cont2, "cont2", "TriggerButton");


    mCont1 = cont1;
    mCont2 = cont2;
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
      return mCont1.getRightTriggerAxis() >= .9 || mCont2.getRightTriggerAxis() >= .9;
    }
    return mCont1.getLeftTriggerAxis() >= .9 || mCont2.getLeftTriggerAxis() >= .9;
  }
}
