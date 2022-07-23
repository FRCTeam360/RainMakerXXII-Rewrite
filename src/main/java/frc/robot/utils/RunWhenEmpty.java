// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Tower;

/** Add your docs here. */
public class RunWhenEmpty implements BooleanProvider{
    Feeder feeder = Feeder.getInstance();
    Tower tower = Tower.getInstance();
    public boolean getBoolean() {
        return !feeder.hasBall() && !tower.bottomHasBall();
    }
}
