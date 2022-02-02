package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
public class IntakeLiftSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  private PWMVictorSPX m_IntakeLiftMotor;
  private DigitalInput m_limitUpSwitch=new DigitalInput(LimitSwitches.ArmUp);
  private DigitalInput m_limitDownSwitch=new DigitalInput(LimitSwitches.ArmDown);
  private Counter m_limitUpCounter=new Counter(m_limitUpSwitch);
  private Counter m_limitDownCounter=new Counter(m_limitDownSwitch);

  public IntakeLiftSubsystem() {
    m_IntakeLiftMotor=new PWMVictorSPX(IntakeConstants.intakeLiftMotorPort);
  }

  public void resetUpperSwitch(){
    m_limitUpCounter.reset();
  }
  public void resetLowerSwitch(){
    m_limitDownCounter.reset();
  }
  public boolean isUpperSwitchSet() {
    System.out.println("Up Counter = " + m_limitUpCounter.get());
    return m_limitUpCounter.get() > 0;
 }

public boolean isLowerSwitchSet() {
  System.out.println("Down Counter = " + m_limitDownCounter.get());
    
  return m_limitDownCounter.get() > 0;
}

  public void setSpeed(double speed){
      m_IntakeLiftMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}