package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;



public class zAutoTargetandMoveCommand extends CommandBase{
private int m_gridChoice=0;
private Limelight m_limeLight;
private int m_pipeline=0;
private double m_targetHeight;
private DrivetrainSubsystem m_drive;
private final MedianFilter m_filter = new MedianFilter(3);
private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.rightKI,PIDSteering.rightKD);
private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.leftKI,PIDSteering.leftKD);
private double kVisionConeTolerance=Constants.LimeLightValues.kVisionXTolerance;
private double kVisionCubeTolerance=kVisionConeTolerance*6; // Wider target for Cubes
private double kGyroTolerance=5;
private double m_visionXTolerance=0;
private double kTimeToMoveForward=.3; // how long do we move forward
private double kTimeToMoveBackward=.05;
private double kMinPIDOutput = 0.02;
private double kMoveSpeed=0.10;
private double m_startMoveTime;
private XboxController m_driver;
private STATE m_state;

private enum STATE{
  MAKESTRAIGHT,MOVEFORWARD,BACKUP,LINEUP,END,ABORT;
}



public zAutoTargetandMoveCommand(Limelight limelight,DrivetrainSubsystem drive,int gridChoice,XboxController driver){
        m_gridChoice=gridChoice;
        m_limeLight=limelight;
        m_drive=drive;
        m_driver=driver;
        addRequirements(m_drive);
        //GRID Choice are 1 to 9 for the three rows in a grid 
        //GRID 10 (top cone any column) 11 (middle cone any column) 12 (botton cone any column)
        //GRID 13 (top cube  column) 14 (middle cube any column), 15 (bottom cube any column)
        switch (gridChoice)
        {
            //fall thorugh cases for same pipelines and heights
            case Constants.ChargedUp.GridPosUpperLeft:
            case Constants.ChargedUp.GridPosUpperRight:
            case Constants.ChargedUp.GridPosUpperConeAny:
                m_pipeline=Constants.VisionPipelines.TopTape;   
                m_targetHeight=Constants.ChargedUp.targetHeightHighTape;
                m_visionXTolerance=kVisionConeTolerance;
                break;
            case Constants.ChargedUp.GridPosMiddleLeft:
            case Constants.ChargedUp.GridPosMiddleRight:
            case Constants.ChargedUp.GridPosMiddleConeAny:
            case Constants.ChargedUp.GridPosBottomLeft:
            case Constants.ChargedUp.GridPosBottomRight:
            case Constants.ChargedUp.GridPosBottomConeAny:
                m_pipeline=Constants.VisionPipelines.BottomTape;
                m_targetHeight=Constants.ChargedUp.targetHeighMLowTape;
                m_visionXTolerance=kVisionConeTolerance;
                break;
            case Constants.ChargedUp.playerStation:
            // will need to be 15 inches to the left of AprilTag
            m_pipeline=Constants.VisionPipelines.PlayerStationTag;
            m_targetHeight=Constants.ChargedUp.targetHeightPlayerStationTag;
            m_visionXTolerance=kVisionCubeTolerance;
            break;
            default:
                // Assume AprilTag if not a Cone Position
                m_pipeline=Constants.VisionPipelines.AprilTag;
                m_targetHeight=Constants.ChargedUp.targetHeightAprilTag;
                m_visionXTolerance=kVisionCubeTolerance;
            
        }
    }
    @Override
    public void initialize() {
      if(m_limeLight.getPipeline()!=m_pipeline){
          System.out.println("AutoMove Switching Pipeline - " + m_pipeline + " from " + m_limeLight.getPipeline());
          m_limeLight.setPipeline(m_pipeline);
          m_limeLight.setTargetHeight(m_targetHeight);
          m_limeLight.update();
    }
      m_state = STATE.LINEUP;
      m_startMoveTime=0;
         //if the command doesnt have the target abort
         if(!m_limeLight.isTargetAvailible()){
          System.out.println("Lost Target - TargetAvailable=" + m_limeLight.isTargetAvailible());
          m_state=STATE.ABORT;
        }
      
    }
  
     @Override
    public void execute() {
     
    }
    
    
  
    @Override
    public void end(boolean interrupted) {

      m_state = STATE.ABORT;
    }
  
    @Override
    public boolean isFinished() {
   
      boolean returnValue = false;
      double measurement;
      double filteredMeasurement; 
      
      //allow driver overiride by moving joystick
      //System.out.println(m_driver.getLeftX() + " " + m_driver.getLeftY());
      
      if((Math.abs(m_driver.getLeftX())>Constants.swerveDrive.driveDeadband*2) || (Math.abs(m_driver.getLeftY())>Constants.swerveDrive.driveDeadband*2)) {
        System.out.println("Driver Control Aboring Targeting");
        m_state=STATE.ABORT;
      }


      m_limeLight.update();
      measurement = m_limeLight.getXRaw();
   
      filteredMeasurement = m_filter.calculate(measurement);
   
      switch(m_state){
        case MAKESTRAIGHT:
          m_state=STATE.LINEUP;
         //TODO: Validate works before reneabling 
         /*
          double currentAngle=m_drive.getGyroAngle();
          if(currentAngle<kGyroTolerance){
            m_drive.stop();
            m_state=STATE.MOVEFORWARD;
          }else{
            if(currentAngle<0){
                m_drive.spinLeft(0.10);
            }else{
                m_drive.spinRight(0.10);
              }
          }
           */

          break;
          case MOVEFORWARD:
             if(m_startMoveTime==0){
                 System.out.println("AutoMove Forward Started");
                 m_startMoveTime=Timer.getFPGATimestamp();
                 m_drive.movenodistance(0,0,kMoveSpeed);               
              } else {
                if(Timer.getFPGATimestamp()>(m_startMoveTime+kTimeToMoveForward)){
                  System.out.println("AutoMove Forward Stopping " + Timer.getFPGATimestamp() + " " + m_startMoveTime);  
                  m_drive.stop();
                  m_startMoveTime=0;
                    m_state=STATE.END;
                }else {
                  m_drive.movenodistance(0,0,kMoveSpeed);               
                }   
                
              }
              break;
              case BACKUP:
              if(m_startMoveTime==0){
                 System.out.println("AutoMove Backward Started");
                 m_startMoveTime=Timer.getFPGATimestamp();
                 m_drive.movenodistance(180,0,kMoveSpeed);               
              } else {
                if(Timer.getFPGATimestamp()>(m_startMoveTime+kTimeToMoveBackward)){
                  System.out.println("AutoMove backward Stopping " + Timer.getFPGATimestamp() + " " + m_startMoveTime);  
                  m_drive.stop();
                  m_startMoveTime=0;
                    m_state=STATE.LINEUP;
                } else{
                  m_drive.movenodistance(180,0,kMoveSpeed);               

                }
              }
              break;
              case LINEUP:
                System.out.println("vision pipeline is " + m_limeLight.getPipeline());
                if(Math.abs(m_limeLight.getXRaw())<m_visionXTolerance){
                  System.out.println("stopping -filterednumber:" + filteredMeasurement + " lastvalue:" + measurement);
                  m_drive.stop();
                  m_state = STATE.MOVEFORWARD; 
                }else{
                  double pidOutput;
                  if(m_limeLight.getXRaw()>0){
                  pidOutput = m_pidRight.calculate(m_limeLight.getXRaw());
                  pidOutput=Math.min(Math.max(pidOutput,-0.10),0.10);
                  System.out.println("AligningR - X is " + m_limeLight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
                
                  m_drive.movenodistance(90,0,pidOutput);
                }else{
                  pidOutput = m_pidLeft.calculate(m_limeLight.getXRaw());
                  pidOutput=Math.min(Math.max(pidOutput,-0.10),0.10);
                  System.out.println("AligningL - X is " + m_limeLight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
              
                  m_drive.movenodistance(270,0,pidOutput);
                }

                //Check to see if PidOutput is to low to move
                if(Math.abs(pidOutput)< kMinPIDOutput){
                  m_drive.stop();
                  m_state = STATE.MOVEFORWARD;
                }
          }
        break;
        case ABORT:
          System.out.println("Aborting zAutoTargetMoveCommand");
        case END:
        m_drive.stop();
        returnValue=true;
      }
      return returnValue; 
  }
    
}
