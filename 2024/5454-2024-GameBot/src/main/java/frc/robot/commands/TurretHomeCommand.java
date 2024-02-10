package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretHomeCommand extends Command {
  private TurretSubsystem m_turret;

  public TurretHomeCommand(TurretSubsystem turret){
    m_turret=turret;
  }

  @Override
  public void end(boolean interrupted){
    m_turret.stop();
  }
  @Override
  public void execute(){
  m_turret.RunTurretMotor(Constants.TurretConstants.hometurretSpeed);

  }

  @Override
  public boolean isFinished(){

    if(m_turret.IsAtHardLimit()){
      m_turret.stop();
      m_turret.SetEncoder(Constants.TurretConstants.homingPosition);
      return true;
    }
    return false;
  }
}
