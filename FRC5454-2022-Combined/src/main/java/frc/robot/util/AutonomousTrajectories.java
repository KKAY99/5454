package frc.robot.util;

import frc.robot.common.control.*;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;
import frc.robot.util.AutonomousPoints.*;

import java.io.IOException;
import java.util.Arrays;

public class AutonomousTrajectories {
    // Use Constants below to manually retrieve values from an auto point
    // private final static int X = 0;
    // private final static int Y = 1;
    // private final static int A = 2;

    // Angles:
    // 0 = front away front drivers
    // 90 = front facing drivers right
    // 180 = front facing drivers
    // 270 = front facing drivers left

    // F1
    // F2
    // F3
    // F4

    // HP_W
    // HP_C
    // HP_E

    // CL_W
    // CL_C
    // CL_E

    // Possible Auto Paths: (* Represents High Priority)
    // * W -> Transit (Don't do anything but move off the tarmac)
    private final Trajectory Wall_Transit_Trajectory;
    // Move distance to edge + Length of robot
    private static final Path Wall_Transit_Path = new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
            .lineTo(new Vector2(50 + 38, 0)).build();

    // * C -> Transit (Don't do anything but move off the tarmac)
    private final Trajectory Center_Transit_Trajectory;
    // Move Safety factor + Length of robot
    private static final Path Center_Transit_Path = new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
            .lineTo(new Vector2(10 + 38, 0)).build();

    // * E -> Transit (Don't do anything but move off the tarmac)
    private final Trajectory Edge_Transit_Trajectory;
    private static final Path Edge_Transit_Path = new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
            .lineTo(new Vector2(10 + 38, 0)).build();
    // Note: These will be the same on both sides for the 3 above

    // * HP_W -> F1
    // TODO: Possibly change final degrees or use arc
    private final Trajectory HP_W_F1_Trajectory;
    private static final Path HP_W_F1_Path = new SimplePathBuilder(Start.HP_W)
            .lineTo(new Vector2(Ball.F1), ShortAngle.Right).build();

    // * HP_W -> F2
    // TODO: Possibly change final degrees or use arc
    private final Trajectory HP_W_F2_Trajectory;
    private static final Path HP_W_F2_Path = new SimplePathBuilder(Start.HP_W)
            .lineTo(new Vector2(Ball.F2), ShortAngle.Backward).build();

    // HP_W -> F1 -> F2
    // HP_W -> F2 -> F1
    // HP_W -> F2 -> F3

    // HP_E -> F1
    // HP_E -> F1 -> F2

    // HP_C -> F2
    // HP_C -> F2 -> F1
    // HP_C -> F2 -> F3

    // * CL_W -> F4
    // TODO: Possibly change final degrees or use arc
    private final Trajectory CL_W_F4_Trajectory;
    private static final Path CL_W_F4_Path = new SimplePathBuilder(Start.CL_W)
            .lineTo(new Vector2(Ball.F4), Rotation2.fromDegrees(225)).build();

    // CL_E -> F4

    // CL_C -> F4

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        Wall_Transit_Trajectory = new Trajectory(Wall_Transit_Path, trajectoryConstraints);
        Center_Transit_Trajectory = new Trajectory(Center_Transit_Path, trajectoryConstraints);
        Edge_Transit_Trajectory = new Trajectory(Edge_Transit_Path, trajectoryConstraints);
        HP_W_F1_Trajectory = new Trajectory(HP_W_F1_Path, trajectoryConstraints);
        HP_W_F2_Trajectory = new Trajectory(HP_W_F2_Path, trajectoryConstraints);
        CL_W_F4_Trajectory = new Trajectory(CL_W_F4_Path, trajectoryConstraints);
    }

    public Trajectory get_Wall_Transit_Trajectory() {
        return Wall_Transit_Trajectory;
    }

    public Trajectory get_Center_Transit_Trajectory() {
        return Center_Transit_Trajectory;
    }

    public Trajectory get_Edge_Transit_Trajectory() {
        return Edge_Transit_Trajectory;
    }

    public Trajectory get_HP_W_F1_Trajectory() {
        return HP_W_F1_Trajectory;
    }

    public Trajectory get_HP_W_F2_Trajectory() {
        return HP_W_F2_Trajectory;
    }

    public Trajectory get_CL_W_F4_Trajectory() {
        return CL_W_F4_Trajectory;
    }
}
