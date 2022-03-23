package frc.robot.util;

import frc.robot.common.control.*;
import frc.robot.common.io.PathReader;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Hashtable;

public class AutonomousTrajectories {

    private enum StartingPoints {
        HumanPlayer_Start,
        HumanPlayer_Center_Start,
        HumanPlayer_Edge_Start,
        Climber_Start,
        Climber_Center_Start,
        Climber_Edge_Start,
    }

    private enum BallPoints {
        Friendly_One,
        Friendly_Two,
        Friendly_Three,
        Friendly_Five,
        Opposing_One,
        Opposing_Two,
        Opposing_Three,
    }

    private Trajectory NO_EXTRA_BALL;
    private static final Path NO_EXTRA_BALL_PATH = new SimplePathBuilder(
            new Vector2(0.0, 0.0), Rotation2.ZERO)
                    .lineTo(new Vector2(0.0, 20.0), Rotation2.ZERO)
                    .build();

    private static final Path ONE_EXTRA_BALL_PATH = new SimplePathBuilder(
            new Vector2(0.0, 0.0), Rotation2.ZERO)
                    .lineTo(new Vector2(0.0, 20.0), Rotation2.ZERO)
                    .build();

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        NO_EXTRA_BALL = new Trajectory(NO_EXTRA_BALL_PATH, trajectoryConstraints);
    }

    public Trajectory getNoBallAuto() {
        return NO_EXTRA_BALL;
    }
}
