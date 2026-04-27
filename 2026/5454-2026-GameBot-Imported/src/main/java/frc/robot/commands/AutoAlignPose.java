package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignPose extends Command {

    private Boolean isAlignRight;
    private CommandSwerveDrivetrain m_swerve;

    public AutoAlignPose(boolean isAlignRight, CommandSwerveDrivetrain drivebase) {
        this.isAlignRight = isAlignRight;
        m_swerve = drivebase;
        addRequirements(drivebase);
    }

    private double flipLR(double value) {
        value = 8.07 - value; //distance from wall (top) of field translated to distance from wall (bottom) of field
        return value;
    }

    @Override
    public void initialize() {

        PathConstraints m_constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

        
        Pose2d startPose = m_swerve.getPose2d();
        startPose=FieldConstants.flipIfRed(startPose);

        Pose2d climbPose = new Pose2d(
                                    isAlignRight ? flipLR(Constants.ClimbConstants.AutoAlign.ySetpoint) : Constants.ClimbConstants.AutoAlign.ySetpoint,
                                    Constants.ClimbConstants.AutoAlign.xSetpoint,
                                    startPose.getRotation()
                                    );

        Command goToClimb = AutoBuilder.pathfindToPose(
            climbPose,
            m_constraints,
            0.0
        );

        SmartDashboard.putString("Asher's Cool Message:","should be running sequence");
          //add to scheduler
        
        Pose2d currentPose = m_swerve.getPose2d();

        if(currentPose.getX()!=0 | currentPose.getY()!=0) {}
            //disable pathing
            //            CommandScheduler.getInstance().schedule(Commands.sequence(goToStart, followAuto));
            CommandScheduler.getInstance().schedule(Commands.sequence(goToClimb));

}
}
