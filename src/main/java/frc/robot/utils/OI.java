// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIPorts;
import frc.robot.utils.TriggerButton.TriggerSide;

/** Add your docs here. */
public class OI {

    public static final ExtendedXboxController driverCont = new ExtendedXboxController(OIPorts.DRIVER_CONT_PORT, 0.125);
    public static final ExtendedXboxController operatorCont = new ExtendedXboxController(OIPorts.OPERATOR_CONT_PORT, 0.125);

    public static final MultiTriggerButton runIntakeButton = new MultiTriggerButton(driverCont, operatorCont, TriggerSide.LEFT);
    public static final MultiJoystickButton runFeederButton = new MultiJoystickButton(driverCont, operatorCont, Button.kLeftBumper.value);
    public static final MultiJoystickButton runTowerButton = new MultiJoystickButton(driverCont, operatorCont, Button.kRightBumper.value);

    public static final MultiJoystickButton extendIntakeButton = new MultiJoystickButton(driverCont, operatorCont, Button.kA.value);
    public static final MultiJoystickButton retractIntakeButton = new MultiJoystickButton(driverCont, operatorCont, Button.kB.value);
    public static final MultiJoystickButton reverseButton = new MultiJoystickButton(driverCont, operatorCont, Button.kX.value);
    public static final MultiJoystickButton queueBallsButton = new MultiJoystickButton(driverCont, operatorCont, Button.kY.value);

    public static final JoystickButton FODReverseButton = new JoystickButton(driverCont, Button.kLeftStick.value);


    public static final POVButton resetButton = new POVButton(driverCont, 0);
    public static final POVButton arcadeButton = new POVButton(driverCont, 90);
    public static final POVButton tankButton = new POVButton(driverCont, 180);
    public static final POVButton fieldOrientedButton = new POVButton(driverCont, 270);
    
}

