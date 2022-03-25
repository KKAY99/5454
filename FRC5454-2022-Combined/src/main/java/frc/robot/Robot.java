package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.math.RigidTransform2;
import frc.robot.common.math.Rotation2;
import frc.robot.common.UpdateManager;
import frc.robot.common.drivers.Limelight;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(
            robotContainer.getDrivetrainSubsystem());

    Command m_autonomousCommand;

    @Override
    public void robotInit() {
        updateManager.startLoop(5.0e-3);
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_autonomousCommand = robotContainer.getAutonomousCommand();

        robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.ON);
        robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
        robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
        robotContainer.disabledPerioidicUpdates();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.ON);

        robotContainer.resetTurret();
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.refreshSmartDashboard();

    }
}
