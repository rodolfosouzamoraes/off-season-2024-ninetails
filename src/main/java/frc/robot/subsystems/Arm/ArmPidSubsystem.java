package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ArmPidSubsystem extends PIDSubsystem {
  private CANSparkMax leftMotorLeader;
  private CANSparkMax rightMotorLeader;
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private Encoder _encoder = new Encoder(Constants.ArmConstants.kChannelA,Constants.ArmConstants.kChannelB,true);
  private final SimpleMotorFeedforward _shooterFeedforward =
      new SimpleMotorFeedforward(
          Constants.ArmConstants.kSVolts, Constants.ArmConstants.kVVoltSecondsPerRotation);
  private final ArmFeedforward _armFeedForward = new ArmFeedforward(0,0.49,1.95,0.11);
  /** Creates a new ArmPidSubsystem. */
  public ArmPidSubsystem() {
    super(new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD));
    getController().setTolerance(Constants.ArmConstants.kShooterToleranceRPS);
    _encoder.setDistancePerPulse(Constants.ArmConstants.kEncoderDistancePerPulse);
    setSetpoint(5);
    m_controller.setIZone(0);
    m_controller.reset();

    leftMotorLeader = new CANSparkMax(Constants.ArmConstants.kLeftMotorLeader, MotorType.kBrushless);
    leftMotorLeader.restoreFactoryDefaults();
    leftMotorLeader.setInverted(true);
    leftMotorLeader.setIdleMode(IdleMode.kBrake);
    leftMotorLeader.setOpenLoopRampRate(0);
    leftMotorLeader.setSmartCurrentLimit(60,80);

    leftMotor = new CANSparkMax(Constants.ArmConstants.kLeftMotor, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    leftMotor.setInverted(true);
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setOpenLoopRampRate(0);
    leftMotor.setSmartCurrentLimit(60,80);
    leftMotor.follow(leftMotorLeader);

    rightMotorLeader = new CANSparkMax(Constants.ArmConstants.kRightMotorLeader, MotorType.kBrushless);
    rightMotorLeader.restoreFactoryDefaults();
    rightMotorLeader.setInverted(false);
    rightMotorLeader.setIdleMode(IdleMode.kBrake);
    rightMotorLeader.setOpenLoopRampRate(0);
    rightMotorLeader.setSmartCurrentLimit(60,80);

    rightMotor = new CANSparkMax(Constants.ArmConstants.kRightMotor, MotorType.kBrushless);
    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(false);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setOpenLoopRampRate(0);
    rightMotor.setSmartCurrentLimit(60,80);
    rightMotor.follow(rightMotorLeader);

    leftMotorLeader.burnFlash();
    leftMotor.burnFlash();
    rightMotorLeader.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    setSetpoint(setpoint);
    leftMotorLeader.setVoltage(m_controller.calculate(getDistance(),setpoint)+_armFeedForward.calculate(setpoint,0.5));
    rightMotorLeader.setVoltage(m_controller.calculate(getDistance(),setpoint)+_armFeedForward.calculate(setpoint,0.5));  
  }

  @Override
  public double getMeasurement() {
    return _encoder.getRate();
  }

  public double getDistance(){
    return _encoder.getDistance();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    leftMotorLeader.set(Constants.ArmConstants.kFeederSpeed);
    rightMotorLeader.set(Constants.ArmConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    leftMotorLeader.setVoltage(0);
    rightMotorLeader.setVoltage(0);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Encoder Distance",_encoder.getDistance());
      SmartDashboard.putNumber("Encoder Mensure",getMeasurement());

  }

  public void resetEncoderArm(){
    _encoder.reset();
  }
}