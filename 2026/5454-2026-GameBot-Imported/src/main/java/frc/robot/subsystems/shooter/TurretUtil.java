package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.TurretStates;
import frc.robot.subsystems.shooter.FieldConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
/**
 * Utility class for turret targeting calculations.
 * All distances and angles are computed from the turret position, not the robot center.
 */
public class TurretUtil {

    // =========================
    // TARGET TYPES
    // =========================

    /** Enum for selecting which lookup table / target to use. */
    public enum TargetType {
        HUB,
        LEFT_PASS,
        RIGHT_PASS,
        NOTARGET
    }

    // =========================
    // DATA CLASS
    // =========================

    /** Immutable snapshot of everything needed to execute a shot. */
    public static class ShotSolution {
        public final double distanceMeters;       // Turret-to-target distance (m)
        public final double turretAngleDegrees;   // Required turret angle relative to robot heading (°)
        public final double trajectoryAngleDegrees; // Pivot / hood angle from lookup table (°)
        public final double shooterSpeedRPS;      // Flywheel speed from lookup table (RPS)
        public final double timeOfFlightSeconds;  // Ball time-of-flight from lookup table (s)
        public final boolean isValid;             // True if shot is within range & turret limits

        public ShotSolution(double distanceMeters, double turretAngleDegrees,
                            double trajectoryAngleDegrees, double shooterSpeedRPS,
                            double timeOfFlightSeconds, boolean isValid) {
            this.distanceMeters = distanceMeters;
            this.turretAngleDegrees = turretAngleDegrees;
            this.trajectoryAngleDegrees = trajectoryAngleDegrees;
            this.shooterSpeedRPS = shooterSpeedRPS;
            this.timeOfFlightSeconds = timeOfFlightSeconds;
            this.isValid = isValid;
        }
    }

    // =========================
    // CONSTANTS & LOOKUP TABLES
    // =========================

    private static final double TURRET_OFFSET_X = Constants.TurretConstants.kTurretOffsetX;
    private static final double TURRET_OFFSET_Y = Constants.TurretConstants.kTurretOffsetY;

    private static final HubLookUpTable hubTable = new HubLookUpTable();
    private static final PassLookUpTable passTable = new PassLookUpTable();

    // =========================
    // TURRET POSE
    // =========================

    /**
     * Returns the field-relative pose of the turret given the current robot pose.
     * The turret heading is set to the robot heading (turret rotation is handled separately).
     */
    public static Pose2d getTurretPose(Pose2d robotPose) {
        return robotPose.plus(new Transform2d(
                new Translation2d(TURRET_OFFSET_X, TURRET_OFFSET_Y),
                new Rotation2d()));
    }

    // =========================
    // TARGET SELECTION
    // =========================

    /** Returns the 2D field pose of the requested target, flipped for red alliance. */
    public static Pose2d getTargetPose(TargetType target) {
        switch (target) {
            case HUB:        return FieldConstants.getHubTarget().toPose2d();
            case LEFT_PASS:  return FieldConstants.getLeftPassTarget().toPose2d();
            case RIGHT_PASS: return FieldConstants.getRightPassTarget().toPose2d();
            default:         return FieldConstants.getHubTarget().toPose2d();
        }
    }

    /**
     * Returns whichever pass target (LEFT_PASS or RIGHT_PASS) is closer to the robot's
     * current turret position.
     */
    public static TargetType getNearestPassTargetType(Pose2d robotPose) {
        Translation2d turret = getTurretPose(robotPose).getTranslation();
        double distLeft  = turret.getDistance(FieldConstants.getLeftPassTarget().toPose2d().getTranslation());
        double distRight = turret.getDistance(FieldConstants.getRightPassTarget().toPose2d().getTranslation());
        return distLeft <= distRight ? TargetType.LEFT_PASS : TargetType.RIGHT_PASS;
    }

    // =========================
    // DISTANCE & ANGLE (from turret)
    // =========================

    /**
     * Horizontal distance in meters from the turret to the target.
     */
    public static double getDistance(Pose2d robotPose, TargetType target) {
        Translation2d turret = getTurretPose(robotPose).getTranslation();
        Translation2d goal = getTargetPose(target).getTranslation();
        return turret.getDistance(goal);
    }

    /**
     * Field-relative angle (radians) from the turret to the target.
     */
    public static double getFieldAngleToTarget(Pose2d robotPose, TargetType target) {
        Translation2d turret = getTurretPose(robotPose).getTranslation();
        Translation2d goal = getTargetPose(target).getTranslation();
        double dx = goal.getX() - turret.getX();
        double dy = goal.getY() - turret.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Turret angle in degrees that the turret must rotate to, relative to the robot's heading.
     * 0° = robot forward, positive = counter-clockwise.
     */
    public static double getTurretAngleDegrees(Pose2d robotPose, TargetType target) {
        double fieldAngleRad = getFieldAngleToTarget(robotPose, target);
        double robotHeadingRad  = robotPose.getRotation().getRadians();
        double turretRad = fieldAngleRad - robotHeadingRad;
        return normalizeDegrees(Math.toDegrees(turretRad));
    }
    public static double get5454TurretAngle(Pose2d robotPose, TargetType target){
        double angle=getTurretAngleDegrees(robotPose, target);
        double ourAngle=0;
        //need to reverse and convert to 360 measure
        if(angle>0){
            ourAngle=360-angle;
        }else {
             ourAngle=Math.abs(angle);
        }
        //System.out.print("Turret Angle Calc:"+ angle +  " Our Angle:" + ourAngle);
        return ourAngle; 
    }
    // =========================
    // LOOKUP TABLE ACCESSORS
    // =========================

    /** Shooter speed (RPS) from the appropriate lookup table. */
    public static double getShooterSpeed(double distanceMeters, TargetType target) {
        return getTableParams(distanceMeters, target).shooterSpeed;
    }

    /** Trajectory / hood angle (degrees) from the appropriate lookup table. */
    public static double getTrajectoryAngle(double distanceMeters, TargetType target) {
        return getTableParams(distanceMeters, target).hoodPosition;
    }

    /** Estimated time-of-flight (seconds) from the appropriate lookup table. */
    public static double getTimeOfFlight(double distanceMeters, TargetType target) {
        return getTableParams(distanceMeters, target).timeOfFlight;
    }

    // =========================
    // COMPLETE SHOT SOLUTION
    // =========================

    /**
     * Computes every parameter needed to take a shot at the given target.
     * All geometry is measured from the turret position.
     *
     * @param robotPose Current robot pose on the field
     * @param target    Which target to shoot at
     * @return A {@link ShotSolution} with all parameters and a validity flag
     */
    public static ShotSolution computeShotSolution(Pose2d robotPose, TargetType target) {
        double dist = getDistance(robotPose, target);
        double turretAngle = getTurretAngleDegrees(robotPose, target);

        var params = getTableParams(dist, target);

        boolean valid = isWithinShootingRange(dist) && isTurretAngleReachable(turretAngle);

        return new ShotSolution(
                dist,
                turretAngle,
                params.hoodPosition,
                params.shooterSpeed,
                params.timeOfFlight,
                valid);
    }

    // =========================
    // SHOOT ON THE MOVE
    // =========================

    /**
     * Computes a "shoot on the move" solution using 5 iterations of virtual-target refinement.
     *
     * <p>Each iteration:
     * <ol>
     *   <li>Uses the current time-of-flight estimate to predict where the turret will be
     *       when the ball arrives (virtual turret position).</li>
     *   <li>Computes the distance from that virtual position to the (stationary) target.</li>
     *   <li>Looks up a new time-of-flight, shooter speed, and trajectory angle for that
     *       virtual distance.</li>
     *   <li>Feeds the new time-of-flight back into the next iteration.</li>
     * </ol>
     * After 5 iterations the solution has converged. The final turret angle is computed from
     * the robot's <em>current</em> pose aiming at the virtual target so the shot compensates
     * for motion during flight.
     *
     * @param robotPose  Current robot field pose
     * @param robotVelX  Robot field-relative X velocity (m/s), e.g. from {@code ChassisSpeeds}
     * @param robotVelY  Robot field-relative Y velocity (m/s), e.g. from {@code ChassisSpeeds}
     * @param target     Which target to shoot at
     * @return A motion-compensated {@link ShotSolution}
     */
    public static ShotSolution computeLeadShotSolution(Pose2d robotPose,
                                                        double robotVelX,
                                                        double robotVelY,
                                                        TargetType target) {
        Translation2d turretNow = getTurretPose(robotPose).getTranslation();
        Translation2d goalTranslation = getTargetPose(target).getTranslation();

        // Seed: static time-of-flight from current turret position
        double tof = getTimeOfFlight(turretNow.getDistance(goalTranslation), target);

        // Iterative virtual-target refinement (5 passes)
        double virtualX = turretNow.getX();
        double virtualY = turretNow.getY();
        HubLookUpTable.ShootingParameters params = null;

        for (int i = 0; i < 5; i++) {
            // Predict turret position when ball arrives
            virtualX = turretNow.getX() + robotVelX * tof;
            virtualY = turretNow.getY() + robotVelY * tof;

            // Distance from virtual turret position to the stationary target
            double virtualDist = new Translation2d(virtualX, virtualY).getDistance(goalTranslation);

            // Look up shot parameters for this virtual distance
            params = getTableParams(virtualDist, target);

            // Refine time-of-flight for next iteration
            tof = params.timeOfFlight;
        }

        // Final virtual distance (from the last iteration's virtual position)
        double finalDist = new Translation2d(virtualX, virtualY).getDistance(goalTranslation);

        // Turret must point from *current* turret position toward the virtual target
        // (the ball is fired now; it arrives at the target when the robot reaches virtualX/Y)
        double dx = goalTranslation.getX() - virtualX;
        double dy = goalTranslation.getY() - virtualY;
        double leadFieldAngle = Math.atan2(dy, dx);
        double turretAngle = normalizeDegrees(
                Math.toDegrees(leadFieldAngle - robotPose.getRotation().getRadians()));

        boolean valid = isWithinShootingRange(finalDist) && isTurretAngleReachable(turretAngle);

        return new ShotSolution(
                finalDist,
                turretAngle,
                params.hoodPosition,
                params.shooterSpeed,
                params.timeOfFlight,
                valid);
    }

    // =========================
    // VALIDATION
    // =========================

    /** True if the distance is within the lookup-table range. */
    public static boolean isWithinShootingRange(double distanceMeters) {
        return distanceMeters >= Constants.TurretConstants.kMinShootingDistance
                && distanceMeters <= Constants.TurretConstants.kMaxShootingDistance;
    }

    /** True if the turret can physically reach the requested angle. */
    public static boolean isTurretAngleReachable(double angleDegrees) {
        return angleDegrees >= Constants.TurretConstants.kMinAngleDegrees
                && angleDegrees <= Constants.TurretConstants.kMaxAngleDegrees;
    }

    // =========================
    // INTERNAL HELPERS
    // =========================

    /** Selects the correct lookup table and returns interpolated parameters. */
    private static HubLookUpTable.ShootingParameters getTableParams(double distanceMeters, TargetType target) {
        switch (target) {
            case HUB:
                return hubTable.getParameters(distanceMeters);
            case LEFT_PASS:
            case RIGHT_PASS:
                // PassLookUpTable has the same ShootingParameters shape; bridge here
                PassLookUpTable.ShootingParameters p = passTable.getParameters(distanceMeters);
                return new HubLookUpTable.ShootingParameters(p.shooterSpeed, p.hoodAngle, p.timeOfFlight);
            default:
                return hubTable.getParameters(distanceMeters);
        }
    }

    /** Normalizes an angle to the range [-180, 180) degrees. */
    private static double normalizeDegrees(double degrees) {
        degrees %= 360.0;
        if (degrees >= 180.0)  degrees -= 360.0;
        if (degrees < -180.0)  degrees += 360.0;
        return degrees;
    }
 
    }