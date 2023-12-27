package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.*;
/** An example command that uses an example subsystem. */
public class zSpinLoadShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ShooterSubsystemVoltage m_shooter;
  private final ConveyorSubsystem m_conveyor;
  private final FeederSubsystem m_feeder;
  private double m_topSpeed;
  private double m_bottomSpeed;
  private double m_minVelocity;
  private boolean m_isFinished=false;
    
  public zSpinLoadShootCommand(ShooterSubsystemVoltage shooter, ConveyorSubsystem conveyor,FeederSubsystem feeder,double topSpeed,double bottomSpeed,double minVelocity) {
    m_shooter=shooter;
    m_conveyor=conveyor;
    m_feeder=feeder;
    m_topSpeed=topSpeed;
    m_bottomSpeed=bottomSpeed;
    m_minVelocity=minVelocity;
    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(feeder);
  }

  public void changeSpeeds(double topSpeed, double bottomSpeed,double minVelocity){
     m_topSpeed=topSpeed;
     m_bottomSpeed=bottomSpeed;
     m_minVelocity=minVelocity;
  }
  
  public double getTopSpeed(){
    return m_topSpeed;
 }

 public double getBottomSpeed(){
  return m_bottomSpeed;
}

public double getMinVelocity(){
  return m_minVelocity;
}
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double startTime=Timer.getFPGATimestamp();
    double currentTime=Timer.getFPGATimestamp();
    m_isFinished=false;
    System.out.println("before speed loop");
    do {
      m_shooter.shoot(m_topSpeed, m_bottomSpeed);
      currentTime=Timer.getFPGATimestamp();
     
    }while(m_shooter.getBottomMotorVelocity()<=m_minVelocity |  m_shooter.getBottomMotorVelocity()<=m_minVelocity);
    System.out.println("outside shot loop");

    startTime=Timer.getFPGATimestamp();
    do {
      m_feeder.run(Constants.FeederSpeed);
      m_conveyor.run(Constants.conveyorUpSpeed);
      currentTime=Timer.getFPGATimestamp();
    }while(currentTime<startTime+Constants.zAutomation.conveyorfeedBall);
    System.out.println("outside conveyeor loop");
    m_conveyor.stop();
    m_feeder.stop();
    m_shooter.stopShooting();
    m_isFinished=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

