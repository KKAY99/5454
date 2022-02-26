// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ClimbSubsystem m_ClimbSubsystem;
    private final double m_speed;
   
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ClimbCommand(ClimbSubsystem Climb,double speed) {
        m_ClimbSubsystem = Climb;
        m_speed=speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_ClimbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        m_ClimbSubsystem.resetlimitCounter();
    }
    
 
    
     // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {     
  //stop on limit only when going down
    if((m_ClimbSubsystem.hitLimit()==false) || (m_speed>0)) {
      m_ClimbSubsystem.run(m_speed);
    } else {
        System.out.println("Limit Switch Hitch" );
         m_ClimbSubsystem.run(0);
    }

  }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_ClimbSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    //if counter has incremented then move up
        return (m_ClimbSubsystem.getLimitCounter()>0);     
        }
}
