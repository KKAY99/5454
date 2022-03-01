// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
/** An example command that uses an example subsystem. */
public class ShooterCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem m_shooterSubsystem;
  private final Limelight m_limelight;
  private final double m_topSpeed;
  private final double m_bottomSpeed;
  private final boolean m_useDistance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooter,Limelight limelight, double defaultTopSpeed,double defaultBottomSpeed,boolean useDistance) {
    m_shooterSubsystem = shooter;
    m_limelight=limelight;
    m_topSpeed=defaultTopSpeed;
    m_bottomSpeed=defaultBottomSpeed;
    m_useDistance=useDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_useDistance) {
      double topSpeed=0;
      double bottomSpeed=0;
      //Calculate Distance using Limelight
      double distance=m_limelight.getDistance();
      if (distance<40){
        topSpeed=650;
        bottomSpeed=750;
      }else if(distance<70){
        topSpeed=700;
        bottomSpeed=750;
      }else if(distance<90){
        topSpeed=750;
        bottomSpeed=800;
      }else if(distance<110){
        topSpeed=850;
        bottomSpeed=950;
      }else if(distance<130){
        topSpeed=1000;
        bottomSpeed=1050;
      }else if(distance<140){
        topSpeed=1100;
        bottomSpeed=1150;
      }else if(distance<150){
        topSpeed=1150;
        bottomSpeed=1200;
      }else if(distance<160){
        topSpeed=1250;
        bottomSpeed=1200;
      }else{
        topSpeed=800;
        bottomSpeed=800;
      } 
      m_shooterSubsystem.shoot(topSpeed, bottomSpeed);

    }else {
      m_shooterSubsystem.shoot(m_topSpeed, m_bottomSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
