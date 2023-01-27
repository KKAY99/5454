
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubSystemHigh;

import frc.robot.subsystems.ElevatorSubSystemLow;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;


public class AutoShootandMoveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ElevatorSubSystemHigh m_highSubsystem ;
  private ElevatorSubSystemLow m_lowSubsystem ;
  private ShooterSubsystem m_shooterSubsystem ;
  private DriveSubsystem m_drive;
  private double m_targetSpeed;
  private double m_driveSpeed;
  private double m_driveDuration;
  private double m_delay;
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public AutoShootandMoveCommand(ElevatorSubSystemHigh highSubsystem,ElevatorSubSystemLow lowSubsystem,ShooterSubsystem shooterSubsystem,double targetSpeed, DriveSubsystem driveSubsystem,double drivespeed,double duration,double delay) {
    m_highSubsystem=highSubsystem;
    m_lowSubsystem=lowSubsystem;
    m_shooterSubsystem=shooterSubsystem;
    m_targetSpeed = targetSpeed;
    m_drive=driveSubsystem;
    m_driveSpeed=drivespeed;
    m_driveDuration=duration;
    m_delay=delay;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_highSubsystem);
    addRequirements(m_lowSubsystem);
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing Combo Shooter");
    //delay
    Timer.delay(m_delay);
    //backup
    m_highSubsystem.setSpeed(-.8);
    m_lowSubsystem.setSpeed(.8);
    Timer.delay(.1);
    //stop  moving start spinning
    m_highSubsystem.setSpeed(0);
    m_lowSubsystem.setSpeed(0);
    m_shooterSubsystem.setSpeed(m_targetSpeed);
    Timer.delay(2);
    //move ball 1 into queue
    m_highSubsystem.setSpeed(.9);
    m_lowSubsystem.setSpeed(-.9);
    Timer.delay(.20);
    //backup 
    m_highSubsystem.setSpeed(-.8);
    m_lowSubsystem.setSpeed(.8);
    Timer.delay(.05);

    //give wheels time to spin up
    m_highSubsystem.setSpeed(0);
    m_lowSubsystem.setSpeed(0);
    Timer.delay(1.2);
    
    //move ball 2 into queue
    m_highSubsystem.setSpeed(.9);
    m_lowSubsystem.setSpeed(-.9);
    Timer.delay(.15);

    //backup 
    m_highSubsystem.setSpeed(-.8);
    m_lowSubsystem.setSpeed(.8);
    Timer.delay(.05);

    //give wheels time to spin up
    m_highSubsystem.setSpeed(0);
    m_lowSubsystem.setSpeed(0);
    Timer.delay(1.2);

    //move ball 3 into queue / longer delay
    m_highSubsystem.setSpeed(.9);
    m_lowSubsystem.setSpeed(-.9);
    Timer.delay(.50);
    //stop shooting now move if moving
    m_highSubsystem.setSpeed(0);
    m_lowSubsystem.setSpeed(0);
    m_shooterSubsystem.setSpeed(0);
    System.out.println("Drive straight Now");
    m_drive.commandDriveStraight(m_driveSpeed,m_driveDuration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_lowSubsystem.setSpeed(0);
    m_highSubsystem.setSpeed(0);
    m_shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
