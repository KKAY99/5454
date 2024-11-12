// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robot;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
  m_robot = new RobotContainer();
    
 
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robot.AllPeriodic();
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_robot.AutonMode();

    // schedule the autonomous command (example)
    m_autonomousCommand=m_robot.getAutonomousCommand();
    if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);   
    }
  } 

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_robot.AutoPeriodic();
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit(){
   // if(m_autonomousCommand != null) {
   //   m_autonomousCommand.cancel();
   // }
    CommandScheduler.getInstance().cancelAll(); // Cancels all commands - will resetdefault command in TelepMode
    m_robot.TeleopMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic(){
    m_robot.TeleopPeriodic();
     
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit(){
    m_robot.DisabledInit();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    //only check during disabled to make sure we don't flip switch by accident in match
    //disabled can error testing
    // m_robot.checkBrakeButton();
    m_robot.DisabledPeriodic();
   
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
