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
  public static PneumaticsModuleType pModule = PneumaticsModuleType.CTREPCM;
 
  public PneumaticsSubsystem(int nodeID,int intakeArmPort,int ClimbPort) {
    
    m_Compressor = new Compressor(nodeID,pModule);      
    m_solenoidIntakeArm = new Solenoid(pModule, intakeArmPort); 
    m_solenoidClimb=new Solenoid(pModule,ClimbPort);
    setEnabled();
    // m_solenoidLatch=new Solenoid(pModule,Constants.Pneumatics.LatchPort);   
  
  }

  public void setClimb(boolean status){
    m_solenoidClimb.set(status);
  }
  public boolean getClimbStatus(){
    return m_solenoidClimb.get();
  }
  public void setIntakeArms(boolean status){
    System.out.println("Setting Arm " + status);
    m_solenoidIntakeArm.set(status);
  
 
  }
  public boolean getIntakeArmStatus(){
    System.out.println("Status - " + m_solenoidIntakeArm.get());
    return m_solenoidIntakeArm.get();
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
