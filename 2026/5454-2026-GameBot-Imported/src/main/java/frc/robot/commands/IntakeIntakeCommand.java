// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private enum foldingStates{
    ROTATE, END
  }
  private foldingStates m_state;
  private IntakeSubsystem m_intake;
  private double m_speed;
  public IntakeIntakeCommand(IntakeSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = -speed;
    m_state=foldingStates.ROTATE;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=foldingStates.ROTATE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("current spike... Stopping Fold");
    m_intake.stopFold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
  boolean returnValue=false;

    
  
  System.out.println("Shooting - State:" + m_state);
    switch(m_state){
    case ROTATE:
        m_intake.outFold(Constants.IntakeConstants.foldSpeed);
        if(m_intake.getFoldState()>Constants.IntakeConstants.ampStop) {
            m_state=foldingStates.END;
        }
    break;
    case END:
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}
