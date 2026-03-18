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
public class IntakeFoldCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  private enum foldingStates{
    ROTATE, END
  }
  private foldingStates m_state;
  private IntakeSubsystem m_intake;
  private double m_speed;
  public IntakeFoldCommand(IntakeSubsystem intake) {
    m_intake = intake;
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
    System.out.println("... Stopping Fold");
    m_intake.stopFold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
  boolean returnValue=false;

    
  
  System.out.println("Shooting - State:" + m_state);
    switch(m_state){
    case ROTATE:
        double foldSpeed=Constants.IntakeConstants.foldSpeed;
        if(!m_intake.isIntakeOutMode()){
          foldSpeed=foldSpeed*-1;  // pull intake in
        } 
        //extend/retract intake
        m_intake.outFold(foldSpeed);
        //check limits
    
        if(!m_intake.isIntakeOutMode()) { //intake 
          //stop intake roller once we are inside limit
          if(m_intake.isinNoFlyZone()){
              m_intake.stopIntake();
          }
          if(m_intake.isAtInLimit()){
            System.out.println("Intake Fold Stoppping Due to In Check");
            
            m_state=foldingStates.END;
          }
       /*    if(m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampInStop)) {
            System.out.println("Intake Fold Stoppping Due to Current Limit Check");
            m_intake.stopFold();
            m_state=foldingStates.END;
        }*/
        }else { //Outake Mode
          //auto restart intake when it is past the no fly zone
          if(!m_intake.isinNoFlyZone()){
              m_intake.runIntake(Constants.IntakeConstants.highSpeed);
          }
          if(m_intake.isAtOutLimit()){
           System.out.println("Intake Fold Stoppping Due to Out Check");
            m_intake.stopFold();
            m_state=foldingStates.END;
          }
       if(m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampOutStop)) {
            System.out.println("Intake Fold Stoppping Due to Current Limit Check");
            m_intake.stopFold();
            m_state=foldingStates.END;
        } }
        
    break;
    case END:
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}
