
package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(new Talon(DriveConstants.kLeftMotor1Port),
                               new Talon(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(new Talon(DriveConstants.kRightMotor1Port),
                               new Talon(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private ADXRS450_Gyro m_gyro;
/*
  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                  DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);
*/
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void tankDrive (double leftSpeed, double  rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  public void commandDriveStraight (double speed,double duration){
    double startTime;
    double currentTime;
    startTime=Timer.getFPGATimestamp();
    currentTime=startTime;
    while( currentTime <= startTime+duration){
      m_leftMotors.set(speed);
      m_rightMotors.set(0-speed);
      currentTime=Timer.getFPGATimestamp();
    }    
    m_leftMotors.set(0);
    m_rightMotors.set(0);
  }
  public void commandTurnRobot (double speed,double direction){
    double startGyro;
    double endGyro;
    startGyro=m_gyro.getAngle();
    endGyro=startGyro + direction;
    System.out.println("TurnRobot " +  speed  + "," + direction);
    System.out.println(endGyro);
    if(direction>0){
      m_leftMotors.set(speed);
      System.out.println("Go Right--" + m_gyro.getAngle() + "," + endGyro);
      while(m_gyro.getAngle()<endGyro)
      {//nothing to do just keep getting angle};
      m_leftMotors.set(speed);
         System.out.println("Go Right-" + m_gyro.getAngle() + "," + endGyro);
      }
      m_leftMotors.set(0);
    }else if(direction<0){
      m_rightMotors.set(speed);
      System.out.println("Go Left--" + m_gyro.getAngle() + "," + endGyro);
            while(m_gyro.getAngle()>endGyro)
      {//nothing to do just keep getting angle};
      m_rightMotors.set(speed);
      System.out.println("Go Left-" + m_gyro.getAngle() + "," + endGyro);
      }
      m_rightMotors.set(0);
    }    
  }
  public void commandDriveTank (double leftSpeed,double rightSpeed,double duration){
    double startTime;
    double currentTime;
    startTime=Timer.getFPGATimestamp();
    currentTime=startTime;
    while( currentTime <= startTime+duration){
      m_leftMotors.set(leftSpeed);
      m_rightMotors.set(rightSpeed);
      currentTime=Timer.getFPGATimestamp();
    }    
    m_leftMotors.set(0);
    m_rightMotors.set(0);
  }
  /**
   * Resets the drive encoders to currently read a position of 0.
   */
//  public void resetEncoders() {
//    m_leftEncoder.reset();
//    m_rightEncoder.reset();
//  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
 // public double getAverageEncoderDistance() {
 //   return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
 // }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
 // public Encoder getLeftEncoder() {
 //   return m_leftEncoder;
 // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
 // public Encoder getRightEncoder() {
 //   return m_rightEncoder;
 // }
  
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  public void setGyro(ADXRS450_Gyro gyro){
    m_gyro=gyro;
  }
}

