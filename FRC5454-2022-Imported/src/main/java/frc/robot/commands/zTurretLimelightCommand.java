// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class zTurretLimelightCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TurretSubsystem m_TurretSubsystem;
  private final Limelight m_limelight;
  private double m_speed;
  private double m_maxSpeed;
  private double m_minSpeed;
  private double m_xTolerance;
  private double m_maxRange;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zTurretLimelightCommand(TurretSubsystem subsystem,Limelight limelight, double speed,double minSpeed, double range, double tolerance) {
    m_TurretSubsystem = subsystem;
    m_limelight=limelight;
    m_speed=speed;
    m_maxSpeed=speed;
    m_minSpeed=minSpeed;
    m_maxRange=range;
    m_xTolerance=tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TurretSubsystem);
  }  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.isTargetAvailible()){
      System.out.println("auto turret x is " + Math.abs(m_limelight.getX())  + " / " + Math.abs(m_limelight.getactualX()) + " - " + m_xTolerance);
     
      if(onTarget()){
           m_TurretSubsystem.stop();
        }else {
          //limit value to max speed
          double speed=Math.min((Math.abs(m_limelight.getX())/m_maxRange)*m_maxSpeed,m_maxSpeed);  
          //limit value to min speed 
          speed=Math.max((Math.abs(m_limelight.getX())/m_maxRange)*m_maxSpeed,m_minSpeed);  
          
          if(m_limelight.getX()>0){
              speed=0-speed; // reverse direction
          }
          System.out.println("speed" + speed);                     
          m_TurretSubsystem.turn(speed);      
        }
      }
      else  {//Find Target 
        System.out.println("Turret Seeking");
        double currentPos=m_TurretSubsystem.getPosition();
        if(m_TurretSubsystem.hasHomed()){
          if(m_TurretSubsystem.hitLeftLimit()){
            m_TurretSubsystem.turn(m_speed);
          }else if (m_TurretSubsystem.hitRightLimit()){
            m_TurretSubsystem.turn(-m_speed);
          }else {
            m_TurretSubsystem.turn(m_speed);
          }
        }

      }
  }

  private boolean onTarget(){
    return Math.abs(m_limelight.getX())<=m_xTolerance;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TurretSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    if (m_TurretSubsystem.hitLeftLimit() && m_TurretSubsystem.isMovingLeft(m_speed)){
      System.out.println("Left Limit");  
      returnValue= true;

    } else if (m_TurretSubsystem.hitRightLimit() && m_TurretSubsystem.isMovingRight(m_speed)){
      System.out.println("Right Limit");  
     
      returnValue= true;
    } else if (onTarget()){
      returnValue=false;  
    }
    return returnValue;
  }
}

