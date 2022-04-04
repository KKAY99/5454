package frc.robot.commands;


import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.classes.Limelight;

import frc.robot.subsystems.*;
/** An example command that uses an example subsystem. */
public class zSpinLoadShootDistanceTimeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ShooterSubsystem m_shooter;
  private final ConveyorSubsystem m_conveyor;
  private final FeederSubsystem m_feeder;
  private final Limelight m_limelight;
  private double m_topSpeed;
  private double m_bottomSpeed;
  private double m_minVelocity;
  private double m_duration;
  private boolean m_isFinished=false;
 
  public zSpinLoadShootDistanceTimeCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor,FeederSubsystem feeder,Limelight limelight,double timeDelay) {
    m_shooter=shooter;
    m_conveyor=conveyor;
    m_feeder=feeder;
    m_limelight=limelight;
    m_duration=timeDelay;
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
    startTime=Timer.getFPGATimestamp();
    do {    
      if (m_shooter.isUptoSpeedbyDistance(m_limelight.getDistance())==false){
          System.out.println("B-" + m_shooter.getBottomMotorVelocity() + " T-" + m_shooter.getTopMotorVelocity());
          m_shooter.shootbyDistance(m_limelight.getDistance()); 
          m_feeder.run(-Constants.FeederSpeed); // Push balls down
          m_conveyor.stop();
      } else {
          m_shooter.shootbyDistance(m_limelight.getDistance()); 
          m_feeder.run(Constants.FeederSpeed);
          m_conveyor.run(Constants.conveyorUpSpeed);

      }
      currentTime=Timer.getFPGATimestamp();
    }while(currentTime<=startTime+m_duration);
    m_isFinished=true;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping zAutoLoadShoot");
    //allow shooter to keep running for auto
      m_conveyor.stop();
      m_feeder.stop();
      m_shooter.stopShooting();
    
    }
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return m_isFinished;
   }
}

