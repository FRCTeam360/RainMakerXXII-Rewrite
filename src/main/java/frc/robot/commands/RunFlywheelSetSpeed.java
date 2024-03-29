// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.metal.MetalBorders.Flush3DBorder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.utils.OI;

public class RunFlywheelSetSpeed extends CommandBase {
  private Flywheel flywheel = Flywheel.getInstance();
  /** Creates a new RunFlywheelSetSpeed. */
  public RunFlywheelSetSpeed() {
    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OI.manualShootFastButton.get()){
      if(OI.reverseButton.get()){
        flywheel.setVelocity(-1000);
      } else {
        flywheel.setVelocity(1500);
      }
    }else if(OI.manualShootSlowButton.get()){
      if(OI.reverseButton.get()){
        flywheel.setVelocity(-600);
      } else {
        flywheel.setVelocity(600);
      }
    }else{
      flywheel.setVelocity(600);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
