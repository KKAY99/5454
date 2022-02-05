package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
/** An example command that uses an example subsystem. */
public class zSpinLoadShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ShooterSubsystem m_shooter;
  private final ConveyorSubsystem m_conveyor;
  private double m_topSpeed;
  private double m_bottomSpeed;
  private boolean m_isFinished=false;
    
  public zSpinLoadShootCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor,double topSpeed,double bottomSpeed) {
    m_shooter=shooter;
    m_conveyor=conveyor;
    m_topSpeed=topSpeed;
    m_bottomSpeed=bottomSpeed;
    addRequirements(shooter);
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double startTime=Timer.getFPGATimestamp();
    double currentTime;
    m_isFinished=false;
       
    do {
      m_shooter.shoot(m_topSpeed, m_bottomSpeed);
      currentTime=Timer.getFPGATimestamp();
     
    }while(currentTime<startTime + Constants.zAutomation.shooterSpinUp);
    System.out.println("outside shot loop");

    startTime=Timer.getFPGATimestamp();
    do {
      m_conveyor.run(Constants.conveyorUpSpeed);
      currentTime=Timer.getFPGATimestamp();
    }while(currentTime<startTime+Constants.zAutomation.conveyorfeedBall);
    System.out.println("outside conveyeor loop");
    m_conveyor.stop();
    m_shooter.stop();
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

