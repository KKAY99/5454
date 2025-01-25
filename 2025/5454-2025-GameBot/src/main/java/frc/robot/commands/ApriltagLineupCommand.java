package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimeLightValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;

public class ApriltagLineupCommand extends Command {
  private CommandSwerveDrivetrain m_swerve;
  private Limelight m_Limelight;

  private double m_targetDistance;
  private double kDriveDeadband=4;
  private double kLineupXDeadband=0.2;
  private double kLineupYawDeadband=1;

  private boolean m_driveTowards = false;
  private boolean m_shouldReset=false;

  private enum States{
    CHECKFORTARGET,DRIVETOWARDS,LINEUP,END
  };

  private States m_currentState = States.CHECKFORTARGET;

  public ApriltagLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight,int fiducialID) {
    m_swerve=swerve;
    m_Limelight=limelight;
    m_driveTowards=false;

    m_Limelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_Limelight.setLimelightIDFilter(fiducialID);
    addRequirements(m_swerve);
  }

  public ApriltagLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight,double distance,int fiducialID) {
    m_swerve=swerve;
    m_Limelight=limelight;
    m_targetDistance=distance;

    if(distance!=0){
      m_driveTowards=true;
    }

    m_Limelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_Limelight.setLimelightIDFilter(fiducialID);
    addRequirements(m_swerve);
  }

  public ApriltagLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight,double distance,int fiducialID,boolean shouldReset) {
    m_swerve=swerve;
    m_Limelight=limelight;
    m_targetDistance=distance;
    m_shouldReset=shouldReset;

    if(distance!=0){
      m_driveTowards=true;
    }

    m_Limelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_Limelight.setLimelightIDFilter(fiducialID);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = States.CHECKFORTARGET;
    //System.out.println("Command started");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    if(m_shouldReset){
      m_Limelight.resetLimelightIDFilter();
    }
    m_currentState = States.CHECKFORTARGET;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double distance=Math.abs(m_Limelight.getDistance());
    double rawX=m_Limelight.getX();
    double x=Math.abs(rawX);
    double rawYaw=m_Limelight.GetYawOfAprilTag();
    double yaw=Math.abs(rawYaw);
    double strafeFlipValue=rawX/x;
    double rotFlipValue=rawYaw/yaw;
    double rotMult=1;
    double strafeMult=1;
    double strafe=1;
    double rotation=1;
    double forward=0;

    switch(m_currentState){
      case CHECKFORTARGET:
        if(m_Limelight.isAnyTargetAvailable()){
          m_currentState=States.LINEUP;
        }else{
          m_currentState=States.END;
        }
      break;
      case DRIVETOWARDS:
        if(distance>(m_targetDistance+LimeLightValues.driveDeadband3)){
          forward=0.8;
        }else if(distance<(m_targetDistance+LimeLightValues.driveDeadband3)&&distance>(m_targetDistance+LimeLightValues.driveDeadband2)){
          forward=0.7;
        }else if(distance<(m_targetDistance+LimeLightValues.driveDeadband2)&&distance>(m_targetDistance+LimeLightValues.driveDeadband1)){
          forward=0.6;
        }else if(distance<(m_targetDistance+LimeLightValues.driveDeadband1)&&distance>(m_targetDistance+LimeLightValues.driveDeadband0)){
          forward=0.5;
        }else if(distance<(m_targetDistance+LimeLightValues.driveDeadband0)){
          forward=0.3;
        }

        if(m_targetDistance<distance+kDriveDeadband){
          m_swerve.drive(forward*-1,0,0);
        }else if(m_targetDistance>distance-kDriveDeadband){
          m_swerve.drive(forward,0,0);
        }

        if(m_targetDistance<distance+kDriveDeadband&&m_targetDistance>distance-kDriveDeadband){
          m_swerve.drive(0,0,0);
          m_currentState = States.END;
        }
      break;
      case LINEUP:
        if(yaw>LimeLightValues.yawLineupDeadband){
          rotMult=1;
        }else{
          rotMult=(yaw/LimeLightValues.yawLineupDeadband)+0.1;
        }

        if(x>LimeLightValues.xLineupDeadband){
          strafeMult=1;
        }else{
          strafeMult=(x/LimeLightValues.xLineupDeadband)+0.1;
        }

        if(x<kLineupXDeadband&&yaw<kLineupYawDeadband){
          m_swerve.drive(0,0,0);
          if(m_driveTowards){
            m_currentState=States.DRIVETOWARDS;
          }else{
            m_currentState=States.END;
          }
        }else{
          if(x<kLineupXDeadband){
            strafe=0;
          }

          if(yaw<kLineupYawDeadband){
            rotation=0;
          }

          m_swerve.drive(0,(strafe*strafeMult)*strafeFlipValue,(rotation*rotMult)*rotFlipValue);
        }
      break;
      case END:
        returnValue=true;
    }

    return returnValue;
  }
}
