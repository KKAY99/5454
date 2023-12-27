// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private PWMVictorSPX m_RightFrontDrive = new PWMVictorSPX(1);
  // private PWMVictorSPX m_RightBackDrive = new PWMVictorSPX(2);
  // private PWMVictorSPX m_LeftFrontDrive = new PWMVictorSPX(8);
  // private PWMVictorSPX m_LeftBackDrive = new PWMVictorSPX(9);

  public ExampleSubsystem() {
  }

   public void run(double speed){
    // m_LeftFrontDrive.set(speed);
    // m_LeftBackDrive.set(speed);
    // m_RightFrontDrive.set(-speed);
    // m_RightBackDrive.set(-speed); 
   }
  
  
  
   public void stop(){
    // m_LeftFrontDrive.set(0);
    // m_LeftBackDrive.set(0);
    // m_RightFrontDrive.set(0);
    // m_RightBackDrive.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
