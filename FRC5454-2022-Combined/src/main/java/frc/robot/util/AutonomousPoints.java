package frc.robot.util;

import frc.robot.common.math.Rotation2;

public class AutonomousPoints {

    public static class StartPoint {
        // Format - Name = {x_Cord in inches, y_Cord in inches, Angle in degrees}

        /** Against Hub on side of human player station */
        public static final double[] HumanPlayer_Wall_Start = { 298.409, 228.698, 111 };

        /**
         * On corner of human player side tarmac,
         * the corner nearest to the center of field
         */
        public static final double[] HumanPlayer_Center_Start = { 248.546, 237.795 };

        /**
         * On corner of human player side tarmac,
         * the corner nearest to the edge of field
         */
        public static final double[] HumanPlayer_Edge_Start = { 298.886, 269.238 };

        /** Against Hub on side of climb station */
        public static final double[] Climber_Wall_Start = { 257.030, 133.106 };

        /**
         * On corner of climb station side tarmac,
         * the corner nearest to the center of field
         */
        public static final double[] Climber_Center_Start = { 216.346, 165.473 };

        /**
         * On corner of climb station side tarmac,
         * the corner nearest to the edge of field
         */
        public static final double[] Climber_Edge_Start = { 226.812, 106.868 };
    }

    public static class Start {
        public static final double[] HP_W = StartPoint.HumanPlayer_Wall_Start;
        public static final double[] HP_C = StartPoint.HumanPlayer_Center_Start;
        public static final double[] HP_E = StartPoint.HumanPlayer_Edge_Start;

        public static final double[] CL_W = StartPoint.Climber_Wall_Start;
        public static final double[] CL_C = StartPoint.Climber_Center_Start;
        public static final double[] CL_E = StartPoint.Climber_Edge_Start;
    }

    // See image linked below for reference
    // F - Friendly
    // O - Opposing
    // Ex: F1 => Friendly_One
    // Ex: O1 => Opposing_One
    // https://cdn.discordapp.com/attachments/272827059456180224/956044537040617502/Field_Diagram.png
    public static class BallPoint {
        // Format - Name = {x_Cord in inches, y_Cord in inches}

        public static final double[] Friendly_One = { 297.978, 311.711 };
        public static final double[] Friendly_Two = { 198.943, 249.224 };
        public static final double[] Friendly_Three = { 41.808, 278.646 };
        public static final double[] Friendly_Four = { 194.493, 79.278 };

        // No need for these... yet...
        // public static final double[] Opposing_One = { 0.0, 0.0 };
        // public static final double[] Opposing_Two = { 0.0, 0.0 };
        // public static final double[] Opposing_Three = { 0.0, 0.0 };
    }

    public static class Ball {
        public static final double[] F1 = BallPoint.Friendly_One;
        public static final double[] F2 = BallPoint.Friendly_Two;
        public static final double[] F3 = BallPoint.Friendly_Three;
        public static final double[] F4 = BallPoint.Friendly_Four;

        // public static final double[] O1 = BallPoint.Opposing_One;
        // public static final double[] O2 = BallPoint.Opposing_Two;
        // public static final double[] O3 = BallPoint.Opposing_Three;
    }

    public static class Angle {
        public static final Rotation2 AwayFromDS = Rotation2.fromDegrees(0.0);
        public static final Rotation2 RightFromDS = Rotation2.fromDegrees(90.0);
        public static final Rotation2 TowardDS = Rotation2.fromDegrees(180.0);
        public static final Rotation2 LeftFromDS = Rotation2.fromDegrees(270.0);
    }

    public static class ShortAngle {
        public static final Rotation2 Forward = Angle.AwayFromDS;
        public static final Rotation2 Right = Angle.RightFromDS;
        public static final Rotation2 Backward = Angle.TowardDS;
        public static final Rotation2 Left = Angle.LeftFromDS;
    }
}
