package frc.robot;

import java.io.File;
import java.nio.file.Path;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.ArmPositionCmd;
import frc.robot.commands.Auto.CollectNoteCmd;
import frc.robot.commands.Auto.ShooterNoteCmd;
import frc.robot.commands.Auto.Auto_ArmPositionCmd;
import frc.robot.commands.Intake.IntakeCollectCmd;
import frc.robot.commands.Intake.IntakeShooterCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.Arm.ArmPidSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
  //#region Subsystem

  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmPidSubsystem m_armPidSubsystem = new ArmPidSubsystem();
  //#endregion
  
  //#region Controle

  private final XboxController m_driverXbox = new XboxController(0);
  private final XboxController m_armIntakeXbox = new XboxController(1);
  //#endregion

  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private Timer m_timer = new Timer();

  public RobotContainer() {
    configureSendableChooser();
    configureBindings();
    configureSwerve();
    configureIntake();
    configureNamedCommand();

  }

  private void configureNamedCommand(){
    //#region Arm

    NamedCommands.registerCommand("Arm Speaker",
    new Auto_ArmPositionCmd(m_armPidSubsystem, 10.3 , 1)
    );

    NamedCommands.registerCommand("Arm Down", 
    new InstantCommand(() -> m_armPidSubsystem.stopFeeder()));

    NamedCommands.registerCommand("Arm LongSpeaker", 
    new Auto_ArmPositionCmd(m_armPidSubsystem, 18 , 1));
    //#endregion

    //#region Intake
    NamedCommands.registerCommand("Collect Note",
    new CollectNoteCmd(m_intakeSubsystem)
    );

    NamedCommands.registerCommand("Shooter Note",
    new ShooterNoteCmd(m_intakeSubsystem)
    .until(() -> m_intakeSubsystem.getSensorCollect() == false));
    //#endregion
  }

  private void configureSendableChooser() {
    autoChooser.addOption("Middle", "Middle");
    autoChooser.addOption("New New Auto", "New New Auto");
    SmartDashboard.putData("autoChooser", autoChooser);

  }

  

  private void configureSwerve() {
    Command baseDriveCommand = m_drivebase.driveCommand(        
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftY()*-1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftX()*-1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getRightX()*0.75*-1,.1));

    m_drivebase.setDefaultCommand(baseDriveCommand);
  }

  private void configureIntake() {
    m_intakeSubsystem.setDefaultCommand(new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem));
  }

  private void configureBindings() {
    initializeIntakeXboxController();
    initializeArmToPositionXboxController();
  }

  private void initializeIntakeXboxController() {
    Trigger shooterNote = new Trigger(() -> Math.abs(m_armIntakeXbox.getRightTriggerAxis())>0.1);
    shooterNote.whileTrue(new IntakeShooterCmd(m_intakeSubsystem));

    Trigger shooterAmp = new Trigger(() -> Math.abs(m_armIntakeXbox.getLeftTriggerAxis())>0.1);
    shooterAmp.whileTrue(new IntakeShooterCmd(m_intakeSubsystem,0.12));

    JoystickButton collectNote = new JoystickButton(m_armIntakeXbox,Button.kRightBumper.value);
    collectNote.whileTrue(new IntakeCollectCmd(m_intakeSubsystem));
  }

  private void initializeArmToPositionXboxController() {
    JoystickButton armMoveArmToSpeakerPosition = new JoystickButton(m_armIntakeXbox,Button.kB.value);
    armMoveArmToSpeakerPosition.onTrue(new ArmPositionCmd(m_armPidSubsystem,10.5,1));

    JoystickButton armMoveArmToSpeakerDistancePosition = new JoystickButton(m_armIntakeXbox,Button.kA.value);
    armMoveArmToSpeakerDistancePosition.onTrue(new ArmPositionCmd(m_armPidSubsystem,17.455,1));

    JoystickButton armMoveArmToAmpPosition = new JoystickButton(m_armIntakeXbox,Button.kY.value);
    armMoveArmToAmpPosition.onTrue(new ArmPositionCmd(m_armPidSubsystem,41.234,1));

    JoystickButton armMoveToDownPosition = new JoystickButton(m_armIntakeXbox,Button.kLeftBumper.value);
    armMoveToDownPosition.onTrue(new InstantCommand(() -> m_armPidSubsystem.stopFeeder(),m_armPidSubsystem));

    JoystickButton resetEncoderArm = new JoystickButton(m_armIntakeXbox,Button.kX.value);
    resetEncoderArm.onTrue(new InstantCommand(() -> m_armPidSubsystem.resetEncoderArm(),m_armPidSubsystem));
  }


  public Command getAutonomousCommand() {
    String autoSelected = autoChooser.getSelected();
    // PathPlannerPath path = PathPlannerPath.fromPathFile(autoChooser.getSelected().toString());
    // m_drivebase.field2d.setRobotPose(new Pose2d(
    //   path.getPathPoses().get(0).getTranslation(),
    //   path.getPathPoses().get(0).getRotation())
    // );

    // return AutoBuilder.followPath(path);
    // return AutoBuilder.buildAuto(path);

    // path.getStartingDifferentialPose();
    
    m_drivebase.setPose(
      // path.getPathPoses().get(0).getTranslation()
      PathPlannerAuto.getStaringPoseFromAutoFile(autoSelected)
      );

      m_drivebase.field2d.setRobotPose(      
      PathPlannerAuto.getStaringPoseFromAutoFile(autoSelected)
      );

    return new PathPlannerAuto(autoSelected);
  }  

  public void setDriveMode() {

  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  private void trashCode(){
    /*
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);


/*     AbsoluteDrive absoluteDrive = new AbsoluteDrive(drivebase,
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                              OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND));
 */
/*
    AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(drivebase,
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                              OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND));
*/
                                                              /*
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY());
    
       */
     //drivebase.setDefaultCommand(absoluteFieldDrive); // Anda somente para frente/atrás e lados, não gira no eixo
  }
}