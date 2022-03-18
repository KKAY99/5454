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

  public static double[] powerTopValues = {
      675,
      725,
      725,
      725,
      775,
      775,
      825,
      875,
      1025,
      1025,
      1125,
      1175,
      1225,
      1325,
      1425,
      25
  };

  public static double[] powerBottomValues = {
      775,
      775,
      775,
      775,
      825,
      825,
      925,
      975,
      1075,
      1075,
      1175,
      1225,
      1275,
      1375,
      1475,
      1075
  };

  public static double[] distanceValues = {
      39.5,
      49.6,
      59.5,
      69.5,
      79.5,
      89.5,
      99.5,
      109.5,
      119.5,
      129.5,
      139.5,
      149.5,
      159.5,
      169.5,
      179.5,
      189.5,
  };

  public static double getPower(double powerValues[], double distance) {
    int i=0;
    try{
      distance = Math.max(distance, 0);
      for (i = 0; i < distanceValues.length; i++) {
        if (distanceValues[i] == distance) {
          return powerValues[i];
        } else if (distanceValues[i] > distance) {
          return getEquation(distance, distanceValues[i], powerValues[i], distanceValues[i - 1],
              powerValues[i - 1]);
        }
        else if (distance > distanceValues[distanceValues.length-1]) {
          return powerValues[distanceValues.length-1];
        }
      }
      return 0;
    } catch(Exception e){
      System.out.println("Exception Error in getpower value i (" + i +  ") " + e.getMessage());
      return 0;
    }
  }

  private static double getEquation(double value, double xOne, double yOne, double xTwo, double yTwo) {
    double slope = (yTwo - yOne) / (xTwo - xOne);
    return (slope * (value - xOne)) + yOne;
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooter, Limelight limelight, double defaultTopSpeed,
      double defaultBottomSpeed, boolean useDistance) {
    m_shooterSubsystem = shooter;
    m_limelight = limelight;
    m_topSpeed = defaultTopSpeed;
    m_bottomSpeed = defaultBottomSpeed;
    m_useDistance = useDistance;
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
    if (m_useDistance) {
      double distance =m_limelight.getDistance();
      double topSpeed = getPower(powerTopValues, distance);
      double bottomSpeed = getPower(powerBottomValues, distance);
      //Default shooting if calculation is outside range
      if(topSpeed<=1 || bottomSpeed <=1){
        topSpeed=m_topSpeed;
        bottomSpeed=m_bottomSpeed;
      }
      m_shooterSubsystem.shoot(topSpeed, bottomSpeed);
    } else {
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
