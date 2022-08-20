// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class MultiJoystickButton extends Button {
  private final GenericHID mJoystick1; 
  private final GenericHID mJoystick2;
  private final int mButtonNumber;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public MultiJoystickButton(GenericHID joystick1, GenericHID joystick2, int buttonNumber) {
    requireNonNullParam(joystick1, "joystick1", "JoystickButton");
    requireNonNullParam(joystick2, "joystick2", "JoystickButton");


    mJoystick1 = joystick1;
    mJoystick2 = joystick2;
    mButtonNumber = buttonNumber;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return mJoystick1.getRawButton(mButtonNumber) || mJoystick2.getRawButton(mButtonNumber);
  }
}
