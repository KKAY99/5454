package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends Command{
  private TurretSubsystem m_turret;
  private IntakeSubsystem m_intake;

  private double m_speed;

  private double m_turretPos;

  private TurretConstants.States m_state;

  public TurretCommand(TurretSubsystem turret,IntakeSubsystem intake,double speed,double pos,TurretConstants.States state){
    m_turret=turret;
    m_intake=intake;
    m_speed=speed;
    m_turretPos=pos;
    m_state=state;
  }

  @Override
  public void execute(){
    m_turret.ResetPIDReference();
  }

  @Override
  public void end(boolean interrupted){
    m_turret.ResetPIDReference();
    m_turret.stop();
    m_intake.stop();
  }

  @Override
  public boolean isFinished(){
    switch(m_state){
      case INTAKE:
      m_turret.TurretSetReference(m_turretPos);
      //run intake
      break;
      case TURRET:
      m_turret.RunCheckLimits(m_speed);
      break;
    }
    return false;
  }
}
