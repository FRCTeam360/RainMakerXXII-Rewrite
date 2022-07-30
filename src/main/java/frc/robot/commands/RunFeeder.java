// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase {
  private Feeder feeder = Feeder.getInstance();
  private boolean reversed;
  /** Creates a new RunFeeder. */
  public RunFeeder(boolean isReversed) {
    addRequirements(feeder);
    reversed = isReversed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public RunFeeder() {
    addRequirements(feeder);
    reversed = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(reversed){
      feeder.run(-1);
    } else {
      feeder.run(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
