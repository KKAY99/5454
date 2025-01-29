package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimeLightValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;

public class ObjectLineupCommand extends Command {
  private CommandSwerveDrivetrain m_swerve;
  private Limelight m_Limelight;

  private double m_targetDistance;
  private double kDeadband = 4;

  private boolean m_driveTowards = false;

  private enum States{
    CHECKFORTARGET,ROTATETOWARDS,DRIVETOWARDS,END
  };

  private States m_currentState = States.CHECKFORTARGET;

  public ObjectLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight) {
    m_swerve=swerve;
    m_Limelight=limelight;

    addRequirements(m_swerve);
  }

  public ObjectLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight,double distance) {
    m_swerve=swerve;
    m_Limelight=limelight;
    m_targetDistance=distance;
    m_driveTowards=true;

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
        if(m_Limelight.isAnyTargetAvailable()){
          m_currentState=States.ROTATETOWARDS;
        }else{
          m_currentState=States.END;
        }
      break;
      case ROTATETOWARDS:

        if(x<0.1){
          m_swerve.drive(0,0,0);
          System.out.println("Stopping");
          if(m_driveTowards){
            m_currentState=States.DRIVETOWARDS;
          }else{
            m_currentState=States.END;
          }
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
      case END:
        returnValue=true;
    }

    return returnValue;
  }
}
