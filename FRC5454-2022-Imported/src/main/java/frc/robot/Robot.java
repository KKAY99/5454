// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoModes;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SendableChooser<Integer> m_autoChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> m_delayChooser = new SendableChooser<Integer>();
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_autoChooser.addOption(AutoModes.autoMode0, AutoModes.autoNothing);
    m_autoChooser.addOption(AutoModes.autoMode1, AutoModes.autoMoveForward);
    m_autoChooser.addOption(AutoModes.autoMode2, AutoModes.autoMoveBackwardshoot);
    m_autoChooser.addOption(AutoModes.autoMode3, AutoModes.autoMoveBackwardsOutake);
    m_autoChooser.addOption(AutoModes.autoMode4, AutoModes.autoMoveBackwardsShot);
    m_autoChooser.addOption(AutoModes.autoMode5, AutoModes.autoMoveShootMoveGrab);
    m_autoChooser.addOption(AutoModes.autoMode6, AutoModes.autoMoveShootMoveGrabShot1);
    m_autoChooser.addOption(AutoModes.autoMode7, AutoModes.autoMoveShotMoveGrabmoveLeftGrabShot2);
    m_autoChooser.addOption(AutoModes.autoMode8, AutoModes.autoMoveShotMoveGrabMoveRightGrabShot);
    m_autoChooser.addOption(AutoModes.autoMode9, AutoModes.autoMoveGrabShot2);
    m_autoChooser.setDefaultOption(AutoModes.autoMode1,AutoModes.autoNothing);
    m_delayChooser.addOption(AutoModes.delayMode0,AutoModes.delayValMode0);
    m_delayChooser.addOption(AutoModes.delayMode1,AutoModes.delayValMode1);
    m_delayChooser.addOption(AutoModes.delayMode2,AutoModes.delayValMode2);
    m_delayChooser.addOption(AutoModes.delayMode3,AutoModes.delayValMode3);
    SmartDashboard.putData("Auto Selector", m_autoChooser);
    SmartDashboard.putData("Delay Time", m_delayChooser);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoChooser.getSelected());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}