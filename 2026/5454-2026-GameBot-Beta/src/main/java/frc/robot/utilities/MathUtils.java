package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.OperatorConstants;
import java.awt.geom.Point2D;

public class MathUtils {
  public static double toUnitCircAngle(double angle) {
    double rotations = angle / (2 * Math.PI);
    return (angle - Math.round(rotations - 0.500) * Math.PI * 2.0);
  }

  public static double cubicLinear(double input, double a, double b) {
    return (a * Math.pow(input, 3) + b * input);
  }

  public static double applyDeadband(double input) {
    if (Math.abs(input) < OperatorConstants.kDeadband) {
      return 0.0;
    } else if (input < 0.0) {
      return (input + OperatorConstants.kDeadband) * (1.0 / (1 - OperatorConstants.kDeadband));
    } else if (input > 0.0) {
      return (input - OperatorConstants.kDeadband) * (1.0 / (1 - OperatorConstants.kDeadband));
    } else {
      return 0.0;
    }
  }

  public static double inputTransform(double input) {
    return cubicLinear(applyDeadband(input), OperatorConstants.kCubic, OperatorConstants.kLinear);
  }

  public static double[] inputTransform(double x, double y) {
    x = applyDeadband(x);
    y = applyDeadband(y);
    double mag = new Translation2d(x, y).getDistance(new Translation2d());

    if (mag > 1.00) {
      mag = 1.00;
    }

    if (mag != 0) {
      x = x / mag * cubicLinear(mag, OperatorConstants.kCubic, OperatorConstants.kLinear);
      y = y / mag * cubicLinear(mag, OperatorConstants.kCubic, OperatorConstants.kLinear);
    } else {
      x = 0;
      y = 0;
    }

    return new double[] { x, y };
  }

  public static double pythagorean(double a, double b) {
    return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
  }

  public static InterpolatingDoubleTreeMap pointsToTreeMap(Point2D[] points) {
    int size = points.length;
    InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();
    for (int i = 0; i < size; i++) {
      m_map.put(points[i].getX(), points[i].getY());
    }
    return m_map;
  }

}
