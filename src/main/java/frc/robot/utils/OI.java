// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIPorts;
import frc.robot.utils.TriggerButton.TriggerSide;

/** Add your docs here. */
public class OI {

    public static final XboxController driverCont = new XboxController(OIPorts.DRIVER_CONT_PORT);
    public static final XboxController operatorCont = new XboxController(OIPorts.OPERATOR_CONT_PORT);

    public static final MultiTriggerButton runIntakeButton = new MultiTriggerButton(driverCont, operatorCont, TriggerSide.LEFT);

    public static final MultiJoystickButton extendIntakeButton = new MultiJoystickButton(driverCont, operatorCont, Button.kA.value);
    public static final MultiJoystickButton retractIntakeButton = new MultiJoystickButton(driverCont, operatorCont, Button.kB.value);
    
}
