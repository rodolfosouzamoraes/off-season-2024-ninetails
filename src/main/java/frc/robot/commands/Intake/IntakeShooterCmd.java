package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeShooterCmd extends Command {
  private final Timer _timer;
  private final IntakeSubsystem _intakeSubsystem;
  private double _speedShooter;
  private double _timerWait = 0.75;
  private double _timerShooter = 0;
  private double _timerFinishShooter = 0;
  private double _timerWaitFinish = 0.5;
  private boolean _isShooter = false;

  public IntakeShooterCmd(IntakeSubsystem intakeSubsystem, double speedShooter) {
    _intakeSubsystem = intakeSubsystem;
    _timer = new Timer();
    _speedShooter = speedShooter;
    addRequirements(intakeSubsystem);
  }

  public IntakeShooterCmd(IntakeSubsystem intakeSubsystem) {
    _intakeSubsystem = intakeSubsystem;
    _timer = new Timer();
    _speedShooter = 1;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.restart();
    _timerShooter = _timer.get() + _timerWait;
    _timerFinishShooter = _timerShooter + _timerWaitFinish;
    _isShooter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_isShooter == true){
      _intakeSubsystem.stopIntake();
      return;
    }
    if(_speedShooter<1){
      _intakeSubsystem.shooterNoteAmp(_speedShooter);
    }
    else{
      _intakeSubsystem.shooterNoteSpeaker(_speedShooter);
    }    
    if(_timerShooter < _timer.get()){
      _intakeSubsystem.pushNote(0.3);
      if(_timerFinishShooter < _timer.get()){
        _isShooter = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
