// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystemVoltage;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
 
/** An example command that uses an example subsystem. */
public class zTurretLimelightFindCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TurretSubsystem m_TurretSubsystem;
  private final Limelight m_limelight;
 
  private double m_speed;
  private double m_maxSpeed;
  private double m_minSpeed;
  private double m_xTolerance;
  private double m_maxRange;
  private CommandState m_state;
  private double m_shooterStart;
  private double m_shooterEnd;
  private double shooterDelay=1;
  public static enum CommandState
    {
        SEARCHING, ONTARGET, SHOOTING;	
    }
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zTurretLimelightFindCommand(TurretSubsystem turret,Limelight limelight, double speed,double minSpeed, double range, double tolerance) {
    m_TurretSubsystem = turret;
//    m_ShooterSubsystem= shooter;
//    m_drive=drive;
    m_limelight=limelight;    
    m_speed=speed;
    m_maxSpeed=speed;
    m_minSpeed=minSpeed;
    m_maxRange=range;
    m_xTolerance=tolerance;
    

    
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TurretSubsystem);
  //  addRequirements(m_ShooterSubsystem);
    //DO NOT DELCARE DRIVE DEPENDENCY
  }  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=CommandState.SEARCHING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
      switch(m_state){
        case ONTARGET:
         if(onTarget() && m_limelight.isTargetAvailible()){ //make sure we are stil on target
          //  System.out.println("Wheelse have Power " + m_drive.wheelsHavePower());
          //  if ((m_drive.wheelsHavePower()==false) &&  (m_TurretSubsystem.isMovingLeft(0)==false) && (m_TurretSubsystem.isMovingRight(0)==false)){
          //    m_shooterStart=Timer.getFPGATimestamp();          
          //    m_state=CommandState.SHOOTING;
          //  }
          } else {
            System.out.println("Setting State to Searching - OnTarget:" + onTarget() + " - TargetAvailable:" + m_limelight.isTargetAvailible());
            m_speed=m_maxSpeed; //RESET SPEED          
            m_state=CommandState.SEARCHING;
          }
          break;
        case SHOOTING:
          /*   fall through to searching since we are removing autoshoot
          if(Timer.getFPGATimestamp()>m_shooterStart+shooterDelay){
              m_ShooterSubsystem.stop();
              System.out.println("Setting State to SEARCHING");
              m_speed=m_maxSpeed; //RESET SPEED
              m_state=CommandState.SEARCHING;
          }
          else{
            m_ShooterSubsystem.shootbyDistance(m_limelight.getDistance(),0,0);
          }
          break;
          */
        case SEARCHING:
          if (m_limelight.isTargetAvailible()){
            if(onTarget()){
                m_TurretSubsystem.stop();
                m_speed=0;
                System.out.println("Setting State to ON TARGET");
                m_state=CommandState.ONTARGET;        
            } else {              
              //limit value to max speed
              double speed=Math.min((Math.abs(m_limelight.getX())/m_maxRange)*m_maxSpeed,m_maxSpeed);  
              //limit value to min speed 
              speed=Math.max((Math.abs(m_limelight.getX())/m_maxRange)*m_maxSpeed,m_minSpeed);  

              if(m_limelight.getX()>0){
                  speed=0-speed; // reverse direction
              }                 
              m_speed=speed;
            }
          } else  {//Find Target 
            if(m_TurretSubsystem.hasHomed()){
              if(m_TurretSubsystem.hitLeftLimit()){
                m_TurretSubsystem.stop();
                System.out.println("Turn Right");
                  m_speed=-m_maxSpeed;
                }else if (m_TurretSubsystem.hitRightLimit()){
                m_TurretSubsystem.stop();
                System.out.println("Turn Left");           
                m_speed=m_maxSpeed;
              }
            
            System.out.println("Turret Seeking " + m_speed);
          }
        
      }
      m_TurretSubsystem.turn(m_speed);
      break;
    }   
      
  }

  private boolean onTarget(){
    return m_limelight.isOnTargetX();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turret Auto Find end");
    m_TurretSubsystem.stop();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.isOnTargetX();
  }
}

