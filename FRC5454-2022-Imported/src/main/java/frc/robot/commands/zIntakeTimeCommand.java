// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class zIntakeTimeCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ConveyorSubsystem m_conveyor;
  private final FeederSubsystem m_feeder;
  private final IntakeSubsystem m_intake;
  private final IntakeSubsystem m_intakeInner;
  private final double m_conveyorSpeed;
  private final double m_intakeOutSpeed;
  private final double m_intakeInnerSpeed;
  private final double m_feederSpeed;

  private final boolean m_keepRunning;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
   
  public zIntakeTimeCommand(IntakeSubsystem intake, IntakeSubsystem intakeInner,double intakeOutSpeed,double intakeInnerSpeed, ConveyorSubsystem conveyor,double conveyorSpeed,FeederSubsystem feeder,double feederSpeed,boolean keepRunning){ 
    m_conveyor=conveyor;
    m_feeder=feeder;
    m_intake=intake;
    m_intakeInner=intakeInner;
    m_conveyorSpeed=conveyorSpeed;
    m_feederSpeed=feederSpeed;
    m_intakeOutSpeed=intakeOutSpeed;
    m_intakeInnerSpeed=intakeInnerSpeed;
    m_keepRunning=keepRunning;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
    addRequirements(m_intakeInner);
    addRequirements(m_conveyor);
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      m_conveyor.run(m_conveyorSpeed);
      m_intake.runIntake(m_intakeOutSpeed);
      m_intakeInner.runIntake(m_intakeInnerSpeed);
      m_feeder.run(m_feederSpeed);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_keepRunning==false){
      m_conveyor.stop();
      m_feeder.stop();
      m_intake.stopIntake();
      m_intakeInner.stopIntake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
