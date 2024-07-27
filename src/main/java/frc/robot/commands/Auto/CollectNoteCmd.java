package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CollectNoteCmd extends Command {
  private final Timer m_timer;
  private final IntakeSubsystem m_intakeSubsystem;

  public CollectNoteCmd(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_timer = new Timer();
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    if(m_intakeSubsystem.getSensorCollect() == false){
      m_intakeSubsystem.collectNota(0.3);
    }
    else{
      m_intakeSubsystem.collectNota(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.getSensorCollect() == true) {
      return true;
    }
    return false;
  }
}