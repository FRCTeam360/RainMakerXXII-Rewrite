// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.utils.OI;

public class ClimbManual extends CommandBase {
  Climber climber = Climber.getInstance();
  Turret turret = Turret.getInstance();
  Flywheel flywheel = Flywheel.getInstance();
  Intake intake = Intake.getInstance();
  /** Creates a new ClimbManual. */
  public ClimbManual() {
    addRequirements(climber, turret, flywheel, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OI.retractClimberButton.get()){
      climber.retractClimber();

    } else if(OI.extendClimberButton.get()){
      climber.extendClimber();

    } else {
      climber.runLeft(-OI.operatorCont.getLeftYWithDeadZone());
      climber.runRight(-OI.operatorCont.getRightYWithDeadZone());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runLeft(0);
    climber.runRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
