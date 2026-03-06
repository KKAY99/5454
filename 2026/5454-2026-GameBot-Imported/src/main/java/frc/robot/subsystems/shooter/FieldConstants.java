package frc.robot.subsystems.shooter;


import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    /** Full length of the FRC field in meters (2026 season) */
    public static final double kFieldLength = 16.5405;

    // SHOOT 3D targets (Blue alliance origin)
    private static final Pose3d hubTargetBlue       = new Pose3d(4.620, 4.040, 3.057144, new Rotation3d());
    private static final Pose3d leftPassTargetBlue  = new Pose3d(2.50, 6.0, 0, new Rotation3d());
    private static final Pose3d rightPassTargetBlue = new Pose3d(2.50, 1.960, 0, new Rotation3d());

    // PATHFIND Pose 2D (Blue alliance origin)
    private static final Pose2d leftTrenchBlue  = new Pose2d(3.650, 7.411, new Rotation2d(Math.PI));
    private static final Pose2d rightTrenchBlue = new Pose2d(3.650, 0.635, new Rotation2d(0));

    private static final Pose2d leftNeutralBlue = new Pose2d(8.2, 7.2, new Rotation2d(-Math.PI / 2));
    private static final Pose2d rightNeutralBlue = new Pose2d(8.2, 0.8, new Rotation2d(Math.PI / 2));

    private static final Pose2d leftOppBlue = new Pose2d(13.4, 7.4, new Rotation2d(0));
    private static final Pose2d rightOppBlue = new Pose2d(13.4, 0.627, new Rotation2d(Math.PI));
    // -------------------------------------------------------------------------
    // Pose2d flip helpers
    // -------------------------------------------------------------------------

    /**
     * Flips a Pose2d to the red alliance side of the field using the same
     * rotational symmetry that PathPlanner uses for 2026:
     *   flipped_X = fieldSizeX - X,  flipped_Y = fieldSizeY - Y
     */
    public static Pose2d flipPose(Pose2d pose) {
        return FlippingUtil.flipFieldPose(pose);
    }

    /**
     * Returns the pose flipped to the red alliance side if currently on red,
     * otherwise returns it unchanged.
     */
    public static Pose2d flipIfRed(Pose2d pose) {
        return isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    // -------------------------------------------------------------------------
    // Pose3d flip helpers
    // -------------------------------------------------------------------------

    /**
     * Flips a Pose3d to the red alliance side of the field using the same
     * rotational symmetry that PathPlanner uses for 2026:
     *   flipped_X = fieldSizeX - X,  flipped_Y = fieldSizeY - Y,  yaw += 180°
     */
    public static Pose3d flipPose3d(Pose3d pose) {
        Rotation3d flipped = new Rotation3d(
            pose.getRotation().getX(),
            pose.getRotation().getY(),
            pose.getRotation().getZ() + Math.PI
        );
        return new Pose3d(
            new Translation3d(
                FlippingUtil.fieldSizeX - pose.getX(),
                FlippingUtil.fieldSizeY - pose.getY(),
                pose.getZ()),
            flipped
        );
    }

    /**
     * Returns the Pose3d flipped to the red alliance side if currently on red,
     * otherwise returns it unchanged.
     */
    public static Pose3d flipIfRed(Pose3d pose) {
        return isRedAlliance() ? flipPose3d(pose) : pose;
    }

    // -------------------------------------------------------------------------
    // Alliance helper
    // -------------------------------------------------------------------------

    /** Returns true if the robot is currently on the red alliance. */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    // -------------------------------------------------------------------------
    // Pose getters (automatically flipped for red alliance)
    // -------------------------------------------------------------------------

    /** Hub (high goal) 3D target pose, automatically flipped for red alliance. */
    public static Pose3d getHubTarget() {
        return flipIfRed(hubTargetBlue);
    }

    /** Left pass 3D target pose, automatically flipped for red alliance. */
    public static Pose3d getLeftPassTarget() {
        return flipIfRed(leftPassTargetBlue);
    }

    /** Right pass 3D target pose, automatically flipped for red alliance. */
    public static Pose3d getRightPassTarget() {
        return flipIfRed(rightPassTargetBlue);
    }

    /** Left trench shoot pose, automatically flipped for red alliance. */
    public static Pose2d getLeftTrenchShoot() {
        return flipIfRed(leftTrenchBlue);
    }

    /** Right trench shoot pose, automatically flipped for red alliance. */
    public static Pose2d getRightTrenchShoot() {
        return flipIfRed(rightTrenchBlue);
    }

    /** Left neutral-zone pose, automatically flipped for red alliance. */
    public static Pose2d getLeftNeutral() {
        return flipIfRed(leftNeutralBlue);
    }

    /** Right neutral-zone pose, automatically flipped for red alliance. */
    public static Pose2d getRightNeutral() {
        return flipIfRed(rightNeutralBlue);
    }

    /** Left opponent-zone pose, automatically flipped for red alliance. */
    public static Pose2d getLeftOpp() {
        return flipIfRed(leftOppBlue);
    }

    /** Right opponent-zone pose, automatically flipped for red alliance. */
    public static Pose2d getRightOpp() {
        return flipIfRed(rightOppBlue);
    }
}
