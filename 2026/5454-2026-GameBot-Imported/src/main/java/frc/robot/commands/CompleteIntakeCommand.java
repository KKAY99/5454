package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;

/** An example command that uses an example subsystem. */
public class CompleteIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private IntakeSubsystem m_intake;
  private HopperSubsystem m_hopper;
  private enum intakeStates{
    INTAKE,PULSE,END
  } 
  private intakeStates m_state;
  private double stateStartTime;
  private final double kSpinUpTime=1;
  private double m_flipSpeed = 1.0;
  private int m_outPulseCount = 0;
  private int m_inPulseCount = 0;
  private final int kOutPulses = 6;  // Number of times to pulse out
  private final int kInPulses = 3;   // Number of times to pulse in
   public CompleteIntakeCommand(IntakeSubsystem intake,HopperSubsystem hopper) {
    m_hopper=hopper;
    m_intake=intake;
    m_state=intakeStates.INTAKE;
    addRequirements(m_hopper);
    addRequirements(m_intake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=intakeStates.INTAKE;
    stateStartTime = Timer.getFPGATimestamp();
    m_outPulseCount = 0;
    m_inPulseCount = 0;
    m_flipSpeed = 1.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  System.out.println("Stopping Intake and Hopper");
 
    m_intake.stopIntake();
    m_intake.stopFold();
    m_hopper.stopAgitate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  boolean returnValue=false;

    
  
    switch(m_state){
    case INTAKE:
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.runIntake(Constants.IntakeConstants.highSpeed); 
        m_state=intakeStates.PULSE; 
    break;
    case PULSE:
        //    System.out.println("Out Pulse Count: " + m_outPulseCount + ", In Pulse Count: " + m_inPulseCount);
        if(m_hopper.getNoFuel()==false){
          //stop agitate when fuel is seen
          m_hopper.stopAgitate();
        }
        if (m_flipSpeed > 0) {
            // Currently pulsing out
            m_intake.outFold(0.2 * m_flipSpeed * Constants.IntakeConstants.foldSpeed);
            m_outPulseCount++;
            if (m_outPulseCount >= kOutPulses) {
                m_outPulseCount = 0;
                m_flipSpeed = -1.0;  // Switch to in direction
            }
        } else {
            // Currently pulsing in
            m_intake.inFold(0.2 * Math.abs(m_flipSpeed) * Constants.IntakeConstants.foldSpeed);
            m_inPulseCount++;
            if (m_inPulseCount >= kInPulses) {
                m_inPulseCount = 0;
                m_flipSpeed = 1.0;  // Switch to out direction
            }
        }
        
       /*  System.out.println("Flip Count: " + m_PulseCount);
        m_PulseCount = m_PulseCount + 1;
        if (m_PulseCount == 5) {
            m_intake.inFold(0.2 * m_flipSpeed);
            m_PulseCount = 0;
            m_flipSpeed = m_flipSpeed * -1;
        }
        if (m_intake.isinNoFlyZone()) {
            m_intake.stopIntake();
        } */

    break;
    case END:
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}