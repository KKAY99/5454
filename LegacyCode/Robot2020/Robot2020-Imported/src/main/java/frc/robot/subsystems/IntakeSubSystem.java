
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubSystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  Spark m_spark = new Spark(IntakeConstants.intakePort);
  public IntakeSubSystem() {
   
  }

  public void setSpeed(double speed){
    m_spark.setSpeed(speed);
    System.out.print("Setting Spark Speed - " + speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
