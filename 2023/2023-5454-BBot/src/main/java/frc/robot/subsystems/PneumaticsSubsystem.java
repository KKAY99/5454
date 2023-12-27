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
  private static Solenoid m_clawSolenoid1;
  private static Solenoid m_clawSolenoid2;
  private static Solenoid m_armExtensionSolenoid;
  private static Solenoid m_armHoldSolenoid;
  public static PneumaticsModuleType pModule = PneumaticsModuleType.CTREPCM;

  public PneumaticsSubsystem(int nodeID,int clawSolenoid1ID,int clawSolenoid2ID, int armExtensionSolenoidID, int armHoldSolenoidID) {
    m_Compressor = new Compressor(nodeID,pModule);
    System.out.print("Compressor created - "+ m_Compressor.isEnabled());
    m_clawSolenoid1=new Solenoid(pModule,clawSolenoid1ID);  
    m_clawSolenoid2=new Solenoid(pModule,clawSolenoid2ID);      
    m_armExtensionSolenoid=new Solenoid(pModule,armExtensionSolenoidID);
    m_armHoldSolenoid=new Solenoid(pModule,armHoldSolenoidID);
  }

 public boolean getExtendedState(){
   return m_armExtensionSolenoid.get();
 } 
  public void setExtensionSolenoid(boolean state){
    m_armExtensionSolenoid.set(state);
  }

  public void setTopClawSolenoid(boolean state){
    System.out.println("Setting Top Claw");
    m_clawSolenoid1.set(state);
  }

  public boolean getTopClawSolenoidState(){
    return m_clawSolenoid1.get();
  }

  public boolean getArmHoldCylender(){
     return m_armHoldSolenoid.get();
  }
  public void setArmHoldCynlinder(boolean state){
    m_armHoldSolenoid.set(state);
  }

  public void setBottomClawSolenoid(boolean state){
   System.out.print("Setting Bottom Claw");
    m_clawSolenoid2.set(state);
  }

  public boolean getBottomClawSolenoidState(){
    return m_clawSolenoid2.get();
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
    //m_pressureSwitch = m_Compressor.getPressureSwitchValue();
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
