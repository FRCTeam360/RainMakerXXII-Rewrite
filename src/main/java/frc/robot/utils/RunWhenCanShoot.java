// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Tower;

/** Add your docs here. */
public class RunWhenCanShoot implements BooleanProvider{
    Flywheel flywheel = Flywheel.getInstance();
    Limelight limelight = Limelight.getInstance();
    Feeder feeder = Feeder.getInstance();
    Tower tower = Tower.getInstance();

    public boolean getBoolean() {
        System.out.println("in run when can shoot");
        return (flywheel.isAtSpeed() && limelight.hasValidTarget()) || (!feeder.hasBall() && !tower.bottomHasBall());
    }
}
