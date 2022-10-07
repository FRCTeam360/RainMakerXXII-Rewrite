// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.autos.*;

/** Add your docs here. */
public class AutoChooser {

  private SendableChooser<Command> autoChooser;

  private final Command hangarLeft2Ball = new HL_2Ball();
  private final Command terminalRight5Ball = new TR_5Ball();

  public AutoChooser(){
    autoChooser.addOption("Hang L 2 Ball", hangarLeft2Ball);
    autoChooser.addOption("Term R 5 Ball", terminalRight5Ball);

    SmartDashboard.putData("Auto Choice", autoChooser);
  }

  public Command getCommand() {
    return autoChooser.getSelected();
  }
}
