package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class ShooterNoteCmd extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final Timer m_timer;

  public ShooterNoteCmd(IntakeSubsystem intakeSubsystem) {
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
    m_intakeSubsystem.shooterNoteSpeaker(1);

    if (m_timer.get() > 0.8) {
      m_intakeSubsystem.pushNote(0.4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  public boolean isFinished() {
    if (m_timer.get() > 1.50 && m_intakeSubsystem.getSensorCollect() == false) {
      return true;
    }
    return false;
  }
}