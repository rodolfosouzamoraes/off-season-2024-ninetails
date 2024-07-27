package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmPidSubsystem;

public class ArmPositionCmd extends Command {
  private final ArmPidSubsystem m_armPidSubsystem;
  private final double m_setPosition;
  private final double m_maxOutput;

  public ArmPositionCmd(ArmPidSubsystem armSubsystem, double setPoint, double maxOutPut) {
    m_armPidSubsystem = armSubsystem;
    m_setPosition = setPoint;
    m_maxOutput = maxOutPut;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
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
    return false;
  }
}