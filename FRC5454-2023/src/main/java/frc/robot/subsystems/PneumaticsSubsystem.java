// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Compressor m_Compressor;
  private static boolean m_pressureSwitch;
  private static Solenoid m_solenoidIntakeArm;
  private static Solenoid m_solenoidClimb;
  private static Solenoid m_solenoidHookCables;
  public static PneumaticsModuleType pModule = PneumaticsModuleType.CTREPCM;
  public PneumaticsSubsystem(int nodeID) {
    
    m_Compressor = new Compressor(nodeID,pModule);      
    m_solenoidIntakeArm = new Solenoid(pModule, Constants.Pneumatics.IntakeArmPort); 
    m_solenoidClimb = new Solenoid (pModule,Constants.Pneumatics.ClimbArmPort);
    m_solenoidHookCables= new Solenoid (pModule,Constants.Pneumatics.HookCablesPort);
    setEnabled();
    // m_solenoidLatch=new Solenoid(pModule,Constants.Pneumatics.LatchPort);   
  
  }

  

  public void setHookCables(boolean status){
    m_solenoidHookCables.set(status);
 
  }
  public boolean getHookCableStatus(){
      return m_solenoidHookCables.get();
  }
  public void setIntakeArms(boolean status){
    System.out.println("Setting Arm " + status);
    m_solenoidIntakeArm.set(status);
  
 
  }
  public boolean getIntakeArmStatus(){
    System.out.println("Status - " + m_solenoidIntakeArm.get());
    return m_solenoidIntakeArm.get();
  }
  
  public boolean getClimbArmStatus(){
    return m_solenoidClimb.get();

  }
  public void setClimbArms(boolean status){
    System.out.println("Setting Climb Arm" + status);
    m_solenoidClimb.set(status);
  }

    public void setEnabled(){
    m_Compressor.enableDigital();
    
  }
  
  public void setDisabled(){
    m_Compressor.disable();
  }
  public boolean getEnabled(){
    return m_Compressor.enabled();
   
  }
  public double getCurrent(){
    return m_Compressor.getCurrent();
  }

  public boolean getPressureSwitch(){
    return m_pressureSwitch;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pressureSwitch = m_Compressor.getPressureSwitchValue();
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
