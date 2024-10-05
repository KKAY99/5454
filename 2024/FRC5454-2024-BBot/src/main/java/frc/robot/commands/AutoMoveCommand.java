package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.swerveDrive;

/** An example command that uses an example subsystem. */
public class AutoMoveCommand extends Command {
  private final DrivetrainSubsystem m_drive;
  private final double m_direction;
  private final double m_distance;
  private final double m_rcw;
  private boolean m_useGyro=false;
  private boolean m_isFinished=false;
  private boolean m_driveStarted=false;
  private double m_startingDistance=0;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})    
  public AutoMoveCommand(DrivetrainSubsystem subsystem,double direction,double distance) {
    m_drive=subsystem;
    m_direction=direction;
    m_distance=distance;
    m_rcw=0; // default value - not passed in
    m_useGyro=false;
  }
  public AutoMoveCommand(DrivetrainSubsystem subsystem,double direction,double rcw,double distance) {
    m_drive=subsystem;
    m_direction=direction;
    m_distance=distance;
    m_rcw=rcw;
    m_useGyro=false;
  }

  public AutoMoveCommand(DrivetrainSubsystem subsystem,double direction) {
    m_drive=subsystem;
    m_direction=0;
    m_distance=0;
    m_rcw=0;
 
    m_useGyro=true;    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingDistance=m_drive.legacyGetDistancefromWheel();
    System.out.println("Starting Distance:" + m_startingDistance);
    m_isFinished=false;
    m_driveStarted=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //just wait for IsFinished
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      m_drive.stopAutoDrive();
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      double distancetoGo=m_distance-Math.abs(m_drive.legacyGetDistancefromWheel()-m_startingDistance);
      if(distancetoGo>0){
        System.out.println("Distance to Go:" + distancetoGo + "Current Distance:"+ Math.abs(m_drive.legacyGetDistancefromWheel()) + "Direction:" + m_direction + " Rotation:" + m_rcw );
        if(m_driveStarted==false){
          //TODO: Need to Fix
        m_drive.movenodistance(m_direction,m_rcw,Constants.AutoConstants.MoveSpeed);
         // m_drive.moveNoDistance(m_direction,m_rcw,Constants.AutoConstants.MoveSpeed);   
          m_driveStarted=true;
        }
      }else{     
        System.out.println("stopping");
        m_drive.stopAutoDrive(); 
        m_isFinished=true;
      } 
   return m_isFinished;
  }
}

