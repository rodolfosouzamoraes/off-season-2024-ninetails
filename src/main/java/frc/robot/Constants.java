package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants
{

  public static final double MAXIMUM_SPEED = 0.5;

  public static final double ROBOT_MASS = 20; //(148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class IntakeConstants {
    public static final int kLeftMotorShooter = 20;
    public static final int kRightMotorShooter = 18;
    public static final int kLeftMotorCollect = 21;
    public static final int kRightMotorCollect = 19;

    public static final int kIrSensorCollect = 0;
  }

  public static class ArmConstants {
    public static final int kLeftMotorLeader = 15;
    public static final int kLeftMotor = 14;
    public static final int kRightMotorLeader = 17;
    public static final int kRightMotor = 16;
    public static final int kChannelA = 2;
    public static final int kChannelB = 3;
    public static final double kFactorConvertionPosition = 0.08;


    public static final int[] kEncoderPorts = new int[] {4, 5};
    public static final boolean kEncoderReversed = false;
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = 0.08;
        // Distance units will be rotations
        //1.0 / kEncoderCPR;

    public static final int kShooterMotorPort = 4;
    public static final int kFeederMotorPort = 5;

    public static final double kShooterFreeRPS = 5300;
    public static final double kShooterTargetRPS = 4000;
    public static final double kShooterToleranceRPS = 2;//50;

    // These are not real PID gains, and will have to be tuned for your specific robot.
    public static final double kP = 0.168;//1;
    public static final double kI = 0.0;
    public static final double kD = 0.04;

    // On a real robot the feedforward constants should be empirically determined; these are
    // reasonable guesses.
    public static final double kSVolts = 0.05;
    public static final double kVVoltSecondsPerRotation =
        // Should have value 12V at free speed...
        12.0 / kShooterFreeRPS;

    public static final double kFeederSpeed = 0.5;
  }
}