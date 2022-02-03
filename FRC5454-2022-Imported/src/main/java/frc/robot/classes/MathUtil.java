package frc.robot.classes;

import static java.lang.Math.*;

/**
 * Most code stolen from
 * https://github.com/retrodaredevil/track-shooter/blob/master/core/src/me/retrodaredevil/game/trackshooter/util/MathUtil.java
 */
public final class MathUtil {
	private MathUtil() {
		throw new UnsupportedOperationException();
	}

	/**
	 *
	 * @param a Number on the left of the MOD
	 * @param b Number on the right of the MOD
	 * @return The modulus of a MOD b where the return value is not negative
	 */
	public static double mod(double a, double b) {
		double r = a % b;
		if (r < 0) {
			r += b;
		}
		return r;
	}

	public static int mod(int a, int b) {
		int r = a % b;
		if (r < 0) {
			r += b;
		}
		return r;
	}

	/**
	 * returns a - b or, when |a - b| > wrap / 2, it finds a quicker way <br/>
	 * <br/>
	 * minChange(1, 5, 4) == 0 <br/>
	 * minChange(1, 5, 5) == 1 <br/>
	 * minChange(5, 1, 5) == -1 <br/>
	 * <br/>
	 * minChange(30, 300, 360) == 90 <br/>
	 * minChange(180, 0, 360) == 180 <br/>
	 * minChange(181, 0, 360) == -179 <br/>
	 * 
	 * @param a    Usually the desired variable to get to
	 * @param b    Usually the current variable to change
	 * @param wrap The number that it "wraps" at. Ex: if wrap is 10, 2 is the same
	 *             as 12
	 * @return A positive or negative number that if added to b is the smallest
	 *         change to get to a.
	 */
	public static double minChange(double a, double b, double wrap) {
		a = mod(a, wrap);
		b = mod(b, wrap);
		double change = a - b;
		if (abs(change) > wrap / 2.0f) {
			if (change < 0) {
				change += wrap;
			} else {
				change -= wrap;
			}
		}
		return change;
	}

	/**
	 * @return return Math.abs(minChange(a, b, wrap));
	 */
	public static double minDistance(double a, double b, double wrap) {
		return abs(minChange(a, b, wrap));
	}

	public static double clamp(double input, double min, double max) {
		if(input < min) {
			return min;
		}
		if(input > max) {
			return max;
		}
		return input;
	}

	/**
	 *
	 * @param wpilibDegrees The degrees to remap to the correct way
	 * @return
	 */
	public static double toEulerDegrees(double wpilibDegrees) {
		return 90 - wpilibDegrees; // == euler degrees
	}

	public static double toWPIDegrees(double eulerDegrees) {
		return 90 - eulerDegrees;
	}

	public static double conservePow(double a, double b) {
		return abs(pow(abs(a), b)) * signum(a);
	}
}