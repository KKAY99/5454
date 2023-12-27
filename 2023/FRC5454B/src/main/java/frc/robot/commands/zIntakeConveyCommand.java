package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
import frc.robot.subsystems.*;
/** An example command that uses an example subsystem. */
public class zIntakeConveyCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ConveyorSubsystem m_conveyor;
  private final FeederSubsystem m_feeder;
  private final IntakeSubsystem m_intake;
  private final double m_conveyorSpeed;
  private final double m_intakeOutSpeed;
  private final double m_feederSpeed;

  public zIntakeConveyCommand(IntakeSubsystem intake, double intakeOutSpeed, ConveyorSubsystem conveyor,double conveyorSpeed,FeederSubsystem feeder,double feederSpeed){
    m_conveyor=conveyor;
    m_feeder=feeder;
    m_intake=intake;
    m_conveyorSpeed=conveyorSpeed;
    m_feederSpeed=feederSpeed;
    m_intakeOutSpeed=intakeOutSpeed;
    addRequirements(conveyor);
    addRequirements(feeder);
    addRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.run(m_conveyorSpeed);
    m_intake.runIntake(m_intakeOutSpeed);
    m_feeder.run(m_feederSpeed);
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    m_conveyor.stop();
    m_feeder.stop();
    m_intake.stopIntake(); 
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

