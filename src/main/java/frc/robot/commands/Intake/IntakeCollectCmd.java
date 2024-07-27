// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCollectCmd extends Command {
  private final Timer _timer;
  private final IntakeSubsystem _intakeSubsystem;
  /** Creates a new IntakeCollectCmd. */
  public IntakeCollectCmd(IntakeSubsystem intakeSubsystem) {
    _intakeSubsystem = intakeSubsystem;
    _timer = new Timer();
    addRequirements(_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intakeSubsystem.collectNote(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
