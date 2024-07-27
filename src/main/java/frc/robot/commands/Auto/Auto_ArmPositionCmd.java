package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmPidSubsystem;

public class Auto_ArmPositionCmd extends Command {
  private final ArmPidSubsystem m_armPidSubsystem;
  private final double m_setPosition;
  private final double m_maxOutput;
  private final Timer m_timer;

  public Auto_ArmPositionCmd(ArmPidSubsystem armSubsystem, double setPoint, double maxOutPut) {
    m_armPidSubsystem = armSubsystem;
    m_setPosition = setPoint;
    m_maxOutput = maxOutPut;
    m_timer = new Timer();
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    m_armPidSubsystem.resetEncoderArm();
  }

  @Override
  public void execute() {
   m_armPidSubsystem.useOutput(m_maxOutput, m_setPosition);      
  }

  @Override
  public void end(boolean interrupted) {
    m_armPidSubsystem.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    if (m_timer.get() > 5) {
      return true;
    }
    return false;
  }
}