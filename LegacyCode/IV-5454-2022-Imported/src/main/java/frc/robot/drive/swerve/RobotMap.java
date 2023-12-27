package frc.robot.drive.swerve;


public class RobotMap {
  // Motor controller channels
  public static final int Drive_FrontLeftRotator = 14;
  public static final int Drive_FrontLeftDriver = 15;
  public static final int Drive_FrontRightRotator = 17;
  public static final int Drive_FrontRightDriver = 16;
  public static final int Drive_RearLeftRotator = 11;
  public static final int Drive_RearLeftDriver = 10;
  public static final int Drive_RearRightRotator = 12;
  public static final int Drive_RearRightDriver = 13;
  public static final int Intake_Rollers = 22;
  public static final int Intake_LowerBelt = 21;
  public static final int Intake_UpperBelt = 23;
  public static final int Turret_Rotator = 35;
  public static final int Turret_Flywheel1 = 30;
  public static final int Turret_Flywheel2 = 31;
  public static final int Turret_HoodServo1 = 0;
  public static final int Turret_HoodServo2 = 1;
  public static final int TurretLEDCount = 12;
  public static final int TurretLEDChannel = 9;
  public static final int ClimberTalonChannel = 40;

  // public static final PidConstants PID_Turret_Flywheel = new PidConstants(0.00001, 0, 4e-3);
  public static final PidConstants PID_Turret_Flywheel = new PidConstants(1e-6, 0.009, 0.0015, 1e-4);
  public static final PidConstants PID_Turret_Rotator = new PidConstants(0.025, 0, 0.009, 0.001);
  public static final PidConstants PID_Drive_SwerveRotatorConstants = new PidConstants(5e-3, 1e-10, 1e-8);

  // Sensors
  public static final int FrontRightDriveEncoder = 3; 
  public static final int FrontLeftDriveEncoder = 2;
  public static final int RearRightDriveEncoder = 0;
  public static final int RearLeftDriveEncoder = 1;

  public static final int Intake_BeamIn = 0; 
  public static final int Intake_BeamOut = 1; 

  // Drive constants
  public static final WheelMap FrontRightWheelMap = new WheelMap(Drive_FrontRightDriver, Drive_FrontRightRotator, FrontRightDriveEncoder, 4031);
  public static final WheelMap FrontLeftWheelMap = new WheelMap(Drive_FrontLeftDriver, Drive_FrontLeftRotator, FrontLeftDriveEncoder, 2255);
  public static final WheelMap RearRightWheelMap = new WheelMap(Drive_RearRightDriver, Drive_RearRightRotator, RearRightDriveEncoder, 735);
  public static final WheelMap RearLeftWheelMap = new WheelMap(Drive_RearLeftDriver, Drive_RearLeftRotator, RearLeftDriveEncoder, 2968);
  public static final double DriveSpeedGain = 1.0;
  public static final double DriveSpeedDeadband = 0.15;
  public static final double Wheelbase = 29.0625; 
  public static final double TrackWidth = 17.5625;
  public static final double DownShiftRatio = 0.7;

  // Turret constants
  public static final int T_Rotate_LeftmostLimit = 1205;
  public static final int T_Rotate_RightmostLimit = 34;
  public static final int T_Rotate_Forward = 3404;
  public static final int T_Angle_LeftmostLimit = -5;
  public static final int T_Angle_RightmostLimit = 263;
  public static final int T_Angle_Forward = 192;

  public static final int TurretRotationLowLimitSwitchChannel = 0;
  public static final int TurretRotationHighLimitSwitchChannel = 0;
  public static final int TurretMinHoodAngle = 34;
  public static final int TurretMaxHoodAngle = 51;

  // Flywheel
  public static final int FlywheelIdleRPMs = 1000;

  public static final int DashboardUpdateInterval = 50;

  // Controls
  public static final int Controls_HoodAdjustmentRatio = 3;
  public static final int Controls_TurretAdjustmentRatio = 10;
}
