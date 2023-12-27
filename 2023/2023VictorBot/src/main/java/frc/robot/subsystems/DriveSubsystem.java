package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;



public class DriveSubsystem extends SubsystemBase {
    // private PWMVictorSPX m_RightFrontDrive = new PWMVictorSPX(1);
    // private PWMVictorSPX m_RightBackDrive = new PWMVictorSPX(2);
    // private PWMVictorSPX m_LeftFrontDrive = new PWMVictorSPX(8);
    // private PWMVictorSPX m_LeftBackDrive = new PWMVictorSPX(9);
  
  
    // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(new WPI_VictorSPX(10),
                               new WPI_VictorSPX(11));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(new WPI_VictorSPX(12),
                               new WPI_VictorSPX(13));

  // The robot's drive
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private boolean m_ReverseMode=false; 
  // The left-side drive encoder
//  private final Encoder m_leftEncoder 
    //  new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
      //            DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
 // private final Encoder m_rightEncoder =
    //  new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
    //              DriveConstants.kRightEncoderReversed);
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
    if(m_ReverseMode==false){
      m_drive.tankDrive(leftSpeed, rightSpeed);
    }
    else {
      m_drive.tankDrive(-leftSpeed, -rightSpeed);
    }
  }
  public void arcadeDrive (double moveSpeed, double  rotation) {  
    System.out.println("arcade mode "+ moveSpeed + " - " + rotation);
     m_drive.arcadeDrive(moveSpeed, rotation); 
  }
  public void commandDriveStraight (double speed,double duration){
    double startTime;
    double currentTime;
    startTime=Timer.getFPGATimestamp();
    currentTime=startTime;
    
    while( currentTime <= startTime+duration){
      m_leftMotors.set(speed);
      m_rightMotors.set(-speed);
      currentTime=Timer.getFPGATimestamp();
    }    
    m_leftMotors.set(0);
    m_rightMotors.set(0);
  }
  public void commandDrive (double leftSpeed,double rightSpeed,double duration){
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
  public void moveTank (double leftSpeed,double rightSpeed){
      m_leftMotors.set(leftSpeed);
      m_rightMotors.set(rightSpeed);
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
   public void setReverseMode(){
     m_ReverseMode=true;
   }
   public void setForwardMode(){
     m_ReverseMode=false;
   }
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  
  
}

