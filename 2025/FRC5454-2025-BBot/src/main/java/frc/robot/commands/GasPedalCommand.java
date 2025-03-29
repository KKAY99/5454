// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GasPedalCommand extends Command {
  private DrivetrainSubsystem m_drivetrain;

  private DoubleSupplier m_speed;
  public GasPedalCommand(DrivetrainSubsystem drivetrain, DoubleSupplier speed) {
    m_drivetrain=drivetrain;
    m_speed = speed;
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setGasPedalMult(1, 1);
  }

  
  @Override
  public boolean isFinished() {
    double gasPedalValue=MathUtil.clamp(Math.abs(m_speed.getAsDouble()), 0.1, 0.5);

    m_drivetrain.setGasPedalMult(gasPedalValue, 0.8);
    return false;
  }
}
