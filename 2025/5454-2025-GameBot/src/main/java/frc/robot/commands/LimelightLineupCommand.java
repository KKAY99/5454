package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimeLightValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;

public class LimelightLineupCommand extends Command {
  private CommandSwerveDrivetrain m_swerve;
  private Limelight m_Limelight;

  private double m_targetDistance;
  private double kDeadband = 4;

  private boolean m_driveTowards = false;

  private enum States{
    CHECKFORTARGET,ROTATETOWARDS,DRIVETOWARDS,LINEUP,END
  };

  private States m_currentState = States.CHECKFORTARGET;

  public LimelightLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight) {
    m_swerve=swerve;
    m_Limelight=limelight;

    addRequirements(m_swerve);
  }

  public LimelightLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight,double distance) {
    m_swerve=swerve;
    m_Limelight=limelight;
    m_targetDistance = distance;
    m_driveTowards = true;

    addRequirements(m_swerve);
  }

  public LimelightLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight,double distance,int fiducialID) {
    m_swerve=swerve;
    m_Limelight=limelight;
    m_targetDistance = distance;

    if(distance!=0){
      m_driveTowards=true;
    }

    m_Limelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_Limelight.setIDFilter(fiducialID);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = States.CHECKFORTARGET;
    //System.out.println("Command started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_currentState = States.CHECKFORTARGET;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double distance=Math.abs(m_Limelight.getDistance());
    double rawX=m_Limelight.getX();
    double x=Math.abs(m_Limelight.getX());
    double flipValue=rawX/x;
    double strafe=0;
    double rotation=0;

    switch(m_currentState){
      case CHECKFORTARGET:
        if(m_Limelight.isTargetAvailible()){
          m_currentState = States.ROTATETOWARDS;
        }else{
          m_currentState = States.END;
        }
      break;
      case ROTATETOWARDS:
        if(x>LimeLightValues.lineupDeadband4){
          rotation=1;
        }else if(x<LimeLightValues.lineupDeadband4&&x>LimeLightValues.lineupDeadband3){
          rotation=0.8;
        }else if(x<LimeLightValues.lineupDeadband3&&x>LimeLightValues.lineupDeadband2){
          rotation=0.5;
        }else if(x<LimeLightValues.lineupDeadband2&&x>LimeLightValues.lineupDeadband1){
          rotation=0.33;
        }else if(x<LimeLightValues.lineupDeadband1&&x>LimeLightValues.lineupDeadband0){
          rotation=0.3;
        }else if(x<LimeLightValues.lineupDeadband0){
          rotation=0.3;
        }

        if(x<0.1){
          m_swerve.drive(0,0,0);
          System.out.println("Stopping");
          m_currentState = States.LINEUP;
        }else{
          System.out.println("Rotating At: "+rotation);
          m_swerve.drive(0,0,rotation*flipValue*-1);
        }

      break;
      case DRIVETOWARDS:
        if (m_targetDistance < distance + kDeadband){
          m_swerve.drive(-0.4,0,0);
        }else if(m_targetDistance > distance - kDeadband){
          m_swerve.drive(0.3,0,0);
        }

        System.out.println("Driving towards");

        if (m_targetDistance < distance + kDeadband && m_targetDistance > distance - kDeadband){
          m_swerve.drive(0,0,0);
          m_currentState = States.END;
        }
      break;
      case LINEUP:
        if(x>LimeLightValues.lineupDeadband4){
          strafe=0.4;
        }else if(x<LimeLightValues.lineupDeadband4&&x>LimeLightValues.lineupDeadband3){
          strafe=0.35;
        }else if(x<LimeLightValues.lineupDeadband3&&x>LimeLightValues.lineupDeadband2){
          strafe=0.2;
        }else if(x<LimeLightValues.lineupDeadband2&&x>LimeLightValues.lineupDeadband1){
          strafe=0.1;
        }else if(x<LimeLightValues.lineupDeadband1&&x>LimeLightValues.lineupDeadband0){
          strafe=0.1;
        }else if(x<LimeLightValues.lineupDeadband0){
          strafe=0.08;
        }

        if(x<0.1){
          m_swerve.drive(0,0,0);
          System.out.println("Stopping");
          if(m_driveTowards){
            m_currentState = States.DRIVETOWARDS;
          }else{
            m_currentState=States.END;
          }
        }else{
          System.out.println("Moving At: "+strafe);
          m_swerve.drive(0,strafe*flipValue,0);
        }
      break;
      case END:
        returnValue=true;
        //System.out.println("Command ended");
    }

    return returnValue;
  }
}
