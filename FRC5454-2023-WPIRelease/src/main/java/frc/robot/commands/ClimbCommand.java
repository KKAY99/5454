// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ClimbSubsystem m_ClimbSubsystem;
    private final PneumaticsSubsystem m_pnuematicsSubsystem;
    private final TurretSubsystem m_turret;
    private final double m_speed;
    
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ClimbCommand(ClimbSubsystem climb,PneumaticsSubsystem pneumatics,TurretSubsystem turret, double speed) {
        m_ClimbSubsystem = climb;
        m_pnuematicsSubsystem=pneumatics;
        m_speed=speed;
        m_turret=turret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_ClimbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        
    }
    
 
    
     // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {     
  //stop on limit only when going down
    //make sure turret is out of way
    m_turret.setLocked(); // disable auto targeting mode
    if(m_turret.isClearofClimber()==false){
        m_turret.movePastSafetyPosition(); // start turret moving left
      } else {
        m_turret.stop(); // make sure turret is not moving before climbing either way
        if(m_ClimbSubsystem.stopForLimit(m_speed)==false){
          System.out.println("running Climb - " + m_speed);
          m_ClimbSubsystem.run(m_speed);
        } else { 
                 System.out.println("Checking For Bottom Limit");
                 AutoDeployOnBottomSwitch();          
                  m_ClimbSubsystem.stop();
                               }

                 
        }
  }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Climb Command Ending / Stopping - Interrupted:" + interrupted);
        AutoDeployOnBottomSwitch();
        m_ClimbSubsystem.stop();
        m_turret.stop();
    }

    private void AutoDeployOnBottomSwitch(){
      System.out.println("Checking For Limit Switch Hit");
      //auto deploy pivot arms when climb bottom is hit
      if (m_ClimbSubsystem.hitBottomLimit()){
        System.out.println("Auto deploy arm");
        m_pnuematicsSubsystem.setClimbArms(true);
      }else {
        System.out.println("Bottom Limit-" + m_ClimbSubsystem.hitBottomLimit());
        System.out.println("Top Limit-" + m_ClimbSubsystem.hitTopLimit());    
      }

    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      boolean stopForLimit=false;
      
      stopForLimit=m_ClimbSubsystem.stopForLimit(m_speed);
      System.out.println("Climber Is Finished check* " + m_speed + " * "  + stopForLimit);
      if (stopForLimit){
          AutoDeployOnBottomSwitch();
      }
      return stopForLimit;
    
  }
}
