// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class ExtendedXboxController extends XboxController {
    double deadZone;
    public ExtendedXboxController(int port) {
        super(port);
    }

    public double getRightXWithDeadzone() {
        return removeDeadzoneValues(this.getRightX());
    }

    public double getLeftXWithDeadZone() {
        return removeDeadzoneValues(this.getLeftX());
    }

    public double getRightYWithDeadZone() {
        return removeDeadzoneValues(this.getRightY());
    }

    public double getLeftYWithDeadZone() {
        return removeDeadzoneValues(this.getLeftY());
    }

    private double removeDeadzoneValues(double value) {
        if(Math.abs(value) < deadZone) {
            return 0;
        } else {
            return value;
        }
    }

    public double getRightXSquared() {
        return getSquaredValue(this.getRightX());
    }

    public double getLeftXSquared() {
        return getSquaredValue(this.getLeftX());
    }

    public double getRightYSquared() {
        return getSquaredValue(this.getRightY());
    }

    public double getLeftYSquared() {
        return getSquaredValue(this.getLeftY());
    }
    
    /*
    * it is big brain time
    */
    private double getSquaredValue(double value) {
        return value * Math.abs(value);
    }
}
