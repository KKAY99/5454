package frc.robot.commands;
import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.classes.Limelight;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
/** An example command that uses an example subsystem. */
public class zSpinLoadShootDistanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ShooterSubsystem m_shooter;
  private final ConveyorSubsystem m_conveyor;
  private final FeederSubsystem m_feeder;
  private final Limelight m_limelight;
  private double m_topSpeed;
  private double m_bottomSpeed;
  private double m_minVelocity;
  private boolean m_isFinished=false;
    
  public zSpinLoadShootDistanceCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor,FeederSubsystem feeder,Limelight limelight) {
    m_shooter=shooter;
    m_conveyor=conveyor;
    m_feeder=feeder;
    m_limelight=limelight;
    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(feeder);
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
    getSpeedsByDistance(); // SETS module level speed info
    System.out.println("before speed loop");
    do {
      m_shooter.shoot(m_topSpeed, m_bottomSpeed);    
    }while(m_shooter.getTopMotorVelocity()<=m_minVelocity |  m_shooter.getTopMotorVelocity()<=m_minVelocity);
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
    m_shooter.stop();
    m_isFinished=true;
  }
  
  private void getSpeedsByDistance(){
    double distance=0;
    double topVelocity=0;
    double bottomVelocity=0;

    distance=m_limelight.getDistance();
    if (distance>0 && distance<10){
      topVelocity=800;
      bottomVelocity=800;
    }else if(distance>10 && distance<12){
      topVelocity=1200;
      bottomVelocity=800;
    }else if(distance>=12 && distance<15){
      topVelocity=1600;
      bottomVelocity=800;
    }else if(distance>=12 && distance<15){
      topVelocity=2000;
      bottomVelocity=800;
    }else if(distance>=12 && distance<15){
      topVelocity=2400;
      bottomVelocity=800;
    }else {
      //replace with derived forumla
      topVelocity=Constants.AutoModes.AutoShotTopSpeed;
      bottomVelocity=Constants.AutoModes.AutoShotBottomSpeed;
    }    
    m_topSpeed=topVelocity;
    m_bottomSpeed=bottomVelocity;
    m_minVelocity=topVelocity*.98; //allow a little variance in speed
    
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

