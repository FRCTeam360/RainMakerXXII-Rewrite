// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class RunTower extends CommandBase {
  private boolean reversed;
  private Tower tower = Tower.getInstance();
  /** Creates a new RunTower. */
  public RunTower(boolean isReversed) {
    reversed = isReversed;
    addRequirements(tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public RunTower(){
    reversed = false;
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(reversed){
      tower.run(-1);
    } else {
      tower.run(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
