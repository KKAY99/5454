package frc.robot.commands;
    
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
 public class zMoveElevatorPIDCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  //private final PaddleSubsystem m_IntakeSubsystem;

  private double m_targetHeight = 0.0;
  private final int m_targetPos;
  private double ktargetTolerance=5;
  private final ElevatorSubsystem m_Elevator;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public zMoveElevatorPIDCommand(ElevatorSubsystem elevator,int targetPos) {
    m_Elevator = elevator;  
    m_targetPos = targetPos;
    switch(targetPos){
      case Constants.Lift.topTape:
      m_targetHeight = Constants.Lift.posHighConeFullLiftStage1;
      break;

      case Constants.Lift.middleTape:
      m_targetHeight = Constants.Lift.posMiddleConeFullLiftStage1;
      break;

      case Constants.Lift.apriltag:
      m_targetHeight = Constants.Lift.posCubeOutofLimelight;
      break;
    }
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.SetPosAndMove(m_targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("MoveElevatorPID Ending Pos is " + m_Elevator.getElevatorPos());
    m_Elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //is not finished until elevator is within tolerance of target
    return (Math.abs(m_Elevator.getElevatorPos()-m_targetHeight)<ktargetTolerance);
 }
 }