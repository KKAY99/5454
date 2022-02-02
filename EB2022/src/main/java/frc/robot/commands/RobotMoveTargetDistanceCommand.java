package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Limelight;
import java.lang.Math;
public class RobotMoveTargetDistanceCommand extends CommandBase{
private DriveSubsystem m_drive;
private Limelight m_LimeLight;
private double m_distance;
private boolean m_ended=false;
public RobotMoveTargetDistanceCommand(DriveSubsystem driveSystem, Limelight targetingLimeLight,double targetDistance)
    {
        m_drive=driveSystem;
        m_LimeLight=targetingLimeLight;
        m_distance=targetDistance;
        addRequirements(m_drive);
    }
       // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }
@Override
  public void execute() {
      double maxSpeed=.40;
      double minSpeed=.25;
      double maxError=20;
      double errMultiplier=0;
      double rightSpeed;
      double leftSpeed;
      double oldSpeed;
      double xValue=0;
      boolean bOnTarget=false;
      
        rightSpeed=0;
        leftSpeed=0;
        oldSpeed=0;
        xValue=m_LimeLight.getX();
        if(xValue<0){
          errMultiplier=Math.abs(xValue)/maxError;
          if (errMultiplier>1){
            errMultiplier=1;
          }
          rightSpeed=-(errMultiplier*(maxSpeed-minSpeed) + minSpeed);
          leftSpeed=minSpeed;
        }else if(xValue>0){
          errMultiplier=xValue/maxError;
          if (errMultiplier>1){
            errMultiplier=1;
          }
          leftSpeed=(errMultiplier*(maxSpeed-minSpeed) + minSpeed);
          rightSpeed=-minSpeed;
        }
        System.out.println(xValue + " - " + " driving " + leftSpeed + " -" + rightSpeed);
        System.out.println(m_LimeLight.getDistance() + " -- " + m_distance);
        if(m_LimeLight.getDistance()<m_distance){
          //reverse drive
          oldSpeed=rightSpeed;
          rightSpeed=-rightSpeed;
          leftSpeed=-oldSpeed;
        }
        System.out.println( xValue + " - " + " driving " + leftSpeed + " -" + rightSpeed);
        m_drive.moveTank(leftSpeed, rightSpeed);
        
        bOnTarget=((Math.abs(xValue)<.1) && (Math.abs(m_LimeLight.getDistance()-m_distance)<1));
       
      }
@Override
  public void end(final boolean interrupted) {
    m_ended=true;
    m_drive.moveTank(0,0);
  }
  // Returns true when the command should end.
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
