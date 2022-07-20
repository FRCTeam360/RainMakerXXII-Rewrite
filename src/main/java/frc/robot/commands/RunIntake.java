// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  private Intake intake = Intake.getInstance();
  private boolean reversed;
  /** Creates a new RunIntake. */
  public RunIntake() {
    addRequirements(intake);
    reversed = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public RunIntake(boolean isReversed){
    addRequirements(intake);
    reversed = isReversed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(reversed){
      intake.run(-1);
    } else {
      intake.run(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
