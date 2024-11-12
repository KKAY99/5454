package frc.robot.swervelib.imu;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import java.util.Optional;

/** SwerveIMU interface for the Pigeon2 */
public class Pigeon2Swerve extends SwerveIMU {

  /** Pigeon2 IMU device. */
  WPI_PigeonIMU imu;
  /** Offset for the Pigeon 2. */
  private Rotation3d offset = new Rotation3d();

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus) {
    try{
      imu = new WPI_PigeonIMU(canid);
      PigeonIMUConfiguration config = new PigeonIMUConfiguration();
      imu.configAllSettings(config);
      imu.setYaw(0); //5454 Add 1/8/24
      SmartDashboard.putData(imu);
    }catch(Exception e){
    }
  }

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   */
  public Pigeon2Swerve(int canid) {
    this(canid, "");
  }

  /** Reset IMU to factory default. */
  @Override
  public void factoryDefault() {
    imu.configFactoryDefault();
  }

  /** Clear sticky faults on IMU. */
  @Override
  public void clearStickyFaults() {
    imu.clearStickyFaults();
  }

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset) {
    this.offset = offset;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d() {
    double[] wxyz = new double[4];
    imu.get6dQuaternion(wxyz);
    return new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d() {
    return getRawRotation3d().minus(offset);
  }
 /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  
  
  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration
   * isn't supported returns empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel() {
    short[] initial = new short[3];
    imu.getBiasedAccelerometer(initial);
    return Optional.of(new Translation3d(initial[0], initial[1], initial[2]).times(9.81 / 16384.0));
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU() {
    return imu;
  }
}
