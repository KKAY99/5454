package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.classes.AprilTags;
public class zAutoDetectandGetCommand extends CommandBase {
   
  private Limelight m_limelight;
  private DrivetrainSubsystem m_drive;
  private int m_elementType;
  private AprilTags m_AprilTags; 
    
    public zAutoDetectandGetCommand(Limelight limelight,DrivetrainSubsystem drive,int elementType){
      m_limelight = limelight;
      m_AprilTags = new AprilTags(m_limelight);
      m_drive = drive;
      m_elementType=elementType;
    }

    @Override
    public void execute() {
     // logic is in isFinished
   
   }  
  
    @Override
    public boolean isFinished(){
        try{
          m_limelight.setPipeline(Constants.VisionPipelines.AprilTag);
        
        int turnAngle=m_AprilTags.getAutoTurn();
        
          m_drive.move(turnAngle, turnAngle, 0, 1, false);
          
          //runintakem
          Thread.sleep(10);

    
          //move back
          m_drive.move(180, 0, 0, 1, false);
    
          //rotate back
          m_drive.move(0, turnAngle, 0, 1, true);
        
          return true;
        }
        catch (Exception e) {
          return true; //finished if interrupted
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}