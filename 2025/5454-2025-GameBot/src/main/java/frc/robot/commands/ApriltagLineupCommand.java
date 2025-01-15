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

  private enum States{
    CHECKFORTARGET,LINEUP,END
  };

  private States m_currentState = States.CHECKFORTARGET;

  public ApriltagLineupCommand(CommandSwerveDrivetrain swerve,Limelight limelight) {
    m_swerve=swerve;
    m_Limelight=limelight;

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = States.CHECKFORTARGET;
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

    switch(m_currentState){
      case CHECKFORTARGET:
        if(m_Limelight.isTargetAvailible()){
          m_currentState = States.LINEUP;
        }else{
          m_currentState = States.END;
        }
      break;
      case LINEUP:
        double distance=m_Limelight.getDistance();
        double rawX=m_Limelight.getXRaw();
        double x=Math.abs(m_Limelight.getXRaw());
        double flipValue=rawX/x;
        double strafe=0;

        System.out.println("CurrentX: "+x);
        System.out.println("FlipValue: "+flipValue);

        if(x>LimeLightValues.lineupDeadband4){
          strafe=0.4;
        }else if(x<LimeLightValues.lineupDeadband4&&x>LimeLightValues.lineupDeadband3){
          strafe=0.35;
        }else if(x<LimeLightValues.lineupDeadband3&&x>LimeLightValues.lineupDeadband2){
          strafe=0.2;
        }else if(x<LimeLightValues.lineupDeadband2&&x>LimeLightValues.lineupDeadband1){
          strafe=0.1;
        }else if(x<LimeLightValues.lineupDeadband1&&x>LimeLightValues.lineupDeadband0){
          strafe=0.08;
        }else if(x<LimeLightValues.lineupDeadband0){
          strafe=0.07;
        }

        if(x<0.1){
          m_swerve.drive(0,0,0);
          System.out.println("Stopping");
          returnValue=true;
          m_currentState=States.END;
        }else{
          System.out.println("Moving At: "+strafe);
          m_swerve.drive(0,strafe*flipValue,0);
        }
      break;
      case END:

    }

    return returnValue;
  }
}
