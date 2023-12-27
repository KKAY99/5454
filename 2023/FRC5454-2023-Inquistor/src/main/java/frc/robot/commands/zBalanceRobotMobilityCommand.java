package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

import javax.lang.model.util.ElementScanner14;

import com.kauailabs.navx.frc.AHRS;

public class zBalanceRobotMobilityCommand extends CommandBase {

  private DrivetrainSubsystem m_drive;
  private double currentTime;
  private NavX m_gyro;
  private double zerolevel =-.77;
  private boolean m_hasTipped = false;
  private double m_waitDelay = 2;
  private double m_startWaitTime = 0;
  private boolean m_waitAfterBalanceShift = false;
  private boolean m_hasNotTippedOnce = true;
  private final MedianFilter m_filter = new MedianFilter(3);
  private PIDController m_pidForward = new PIDController(Constants.PIDSteering.forwardKP,PIDSteering.forwardKI,PIDSteering.forwardKD);
  private PIDController m_pidBack = new PIDController(Constants.PIDSteering.backwardKP,PIDSteering.backwardKI,PIDSteering.backwardKD);
  private boolean GetOnRamp=true;
  private boolean tipped=false;
  private boolean useGyro=false;
  private boolean bReversed=false;
  private boolean m_resetStartTime=false;
  private double ktimeToWait=0.25;
  private double m_startTime=0;
  private double m_TimeoutStart;
  private double m_TimeoutCurrent;
  private double kTimeOutCommand=14;

  /** Creates a new testCommand. */
  public zBalanceRobotMobilityCommand(NavX gyro, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro = gyro;
    m_drive = drive;

    addRequirements(drive);
  }

  public void setLevel(double level) {
    zerolevel = level;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_TimeoutStart=Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    double kLowBalanceAngleThresholdDegrees  = -0.1;
    double kHighBalanceAngleThresholdDegrees  = 0.1;


     double xAxisRate            = 0;
     double yAxisRate            = 0;
     double pitchAngleDegrees    = m_gyro.getAxis(Axis.PITCH);
     double rollAngleDegrees     = m_gyro.getAxis(Axis.ROLL);
     boolean m_balanced=false;

     m_TimeoutCurrent=Timer.getFPGATimestamp();


     System.out.println("Checking Balance IsFinished - Pitch " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);
     
     if(GetOnRamp && tipped==false) {
      System.out.println("Drive On " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);
      m_drive.moveGyroNodistance(0,0,0.6);
     }
     if(tipped==false && (rollAngleDegrees<-.15)){
      tipped=true;
      GetOnRamp=false;
      System.out.println("Leave Mobility Zone");
      m_drive.moveGyro(0, 0,0.5,137,true);
      m_drive.stop();
      System.out.println("Reversing Auto");
      //Direction is zero because it is not using the gyro
      m_drive.moveGyroNodistance(180,0,0.60);
      bReversed=true;
     }       
     if(rollAngleDegrees>.15 && bReversed && useGyro==false){
        tipped=true;

        System.out.println("Tipped - Pitch " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);

        m_drive.moveGyro(180, 0,0.35,40,true); // was 40
        System.out.println("Step 2 - Pitch " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);
        m_drive.moveGyro(180, 0,0.25,23.5,true);
        System.out.println("Step 3 - Pitch " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);
        m_drive.moveGyro(0, 0,0.2,8,true);
        System.out.println("Step 4 - Pitch " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);
        useGyro=true;
        bReversed=false;
        m_drive.stop();
     }

     if(useGyro){
      double currentTime=Timer.getFPGATimestamp();
      if(m_resetStartTime){
        m_startTime=Timer.getFPGATimestamp();
        m_resetStartTime=false;
      }
      if(currentTime>m_startTime+ktimeToWait){
        m_resetStartTime=true; // reset time delay
        if(rollAngleDegrees>0.08){
            if(rollAngleDegrees>0.12){
              System.out.println("L-Moving towards player station " + rollAngleDegrees);
            m_drive.moveGyro(180,0,0.15,4,true);
            } else{ 
              System.out.println("S-Moving towards player station " + rollAngleDegrees);
              m_drive.moveGyro(180,0,0.15,3,true);
            }
            
        }else{
          if(rollAngleDegrees<-0.08){
            if(rollAngleDegrees<-0.12){
              System.out.println("L-Moving towards center of field " + rollAngleDegrees);
              m_drive.moveGyro(0, 0, 0.15,4,true);
            } else {
            System.out.println("S-Moving towards center of field " + rollAngleDegrees);
            m_drive.moveGyro(0, 0, 0.15,3,true);
            }
          }else{
            m_drive.stop();
            useGyro=false;
            m_balanced=true;
            System.out.println("Robot is Balanced");
          }
      }
     }
    }
    
    if(m_balanced){
      return true;
     } else {
      if(m_TimeoutCurrent>m_TimeoutStart+kTimeOutCommand){
        System.out.println("Exiting IsFinished due to Timeout");
        return true; //timeout to make sure it ends
         } else{
          return false;
         }
 }    }
}
