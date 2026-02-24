package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystemOverlap extends SubsystemBase {

  private final TalonFX m_turretMotor;
  private final CANcoder m_encoder1;
  private final CANcoder m_encoder2;

  public TurretSubsystemOverlap(int motorCanId, int encoder1Id, int encoder2Id) {
    m_turretMotor = new TalonFX(motorCanId);
    m_encoder1 = new CANcoder(encoder1Id);
    m_encoder2 = new CANcoder(encoder2Id);

    // Keep your existing direction setup (edit if needed)
    CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
    encoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_encoder1.getConfigurator().apply(encoder1Config);

    CANcoderConfiguration encoder2Config = new CANcoderConfiguration();
    encoder2Config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_encoder2.getConfigurator().apply(encoder2Config);
  }

  public TurretSubsystemOverlap() {
    this(TurretConstants.turretCanID, TurretConstants.encoder1CANID, TurretConstants.encoder2CANID);
  }

  public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {

    final double minDeg = TurretConstants.MIN_ROT_DEG;
    final double maxDeg = TurretConstants.MAX_ROT_DEG;
    final double stepDeg = TurretConstants.ABSOLUTE_SOLVE_STEP_DEG;

    java.util.function.DoubleUnaryOperator wrap0to360 = (deg) -> {
      deg %= 360.0;
      if (deg < 0) deg += 360.0;
      return deg;
    };

    java.util.function.BiFunction<Double, Double, Double> signedErrDeg = (aDeg, bDeg) -> {
      double d = wrap0to360.applyAsDouble(aDeg - bDeg);
      if (d > 180.0) d -= 360.0;
      return d;
    };

    java.util.function.Function<Double, Double> enc1Phase = (rawDeg) -> {
      double x = rawDeg - TurretConstants.ENC1_FORWARD_OFFSET_DEG;
      if (TurretConstants.ENC1_INVERT) x = -x;
      return wrap0to360.applyAsDouble(x);
    };

    java.util.function.Function<Double, Double> enc2Phase = (rawDeg) -> {
      double x = rawDeg - TurretConstants.ENC2_FORWARD_OFFSET_DEG;
      if (TurretConstants.ENC2_INVERT) x = -x;
      return wrap0to360.applyAsDouble(x);
    };

    final double enc1 = enc1Phase.apply(e1);
    final double enc2 = enc2Phase.apply(e2);

    double bestTheta = 0.0;
    double bestCost = Double.POSITIVE_INFINITY;

    for (double theta = minDeg; theta <= maxDeg; theta += stepDeg) {

      double pred1 = wrap0to360.applyAsDouble(theta * TurretConstants.ENC1_RATIO);
      double pred2 = wrap0to360.applyAsDouble(theta * TurretConstants.ENC2_RATIO);

      double d1 = signedErrDeg.apply(enc1, pred1);
      double d2 = signedErrDeg.apply(enc2, pred2);

      double cost = (d1 * d1) * 1.5 + (d2 * d2);

      if (cost < bestCost) {
        bestCost = cost;
        bestTheta = theta;
      }
    }

    System.out.println("Turret/SolveEnc1PhaseDeg: " + enc1);
    System.out.println("Turret/SolveEnc2PhaseDeg: " + enc2);
    System.out.println("Turret/SolveBestCost: " + bestCost);

    return bestTheta;
  }

  public void showEncoderPositions() {
    double e1Degrees = m_encoder1.getAbsolutePosition().getValueAsDouble() * 360.0;
    double e2Degrees = m_encoder2.getAbsolutePosition().getValueAsDouble() * 360.0;

    SmartDashboard.putNumber("Encoder 1 ANG", e1Degrees);
    SmartDashboard.putNumber("Encoder 2 ANG", e2Degrees);

    double angle = calculateTurretAngleFromCANCoderDegrees(e1Degrees, e2Degrees);
    SmartDashboard.putNumber("Turret Angle", angle);
  }

  public void moveTurret(double speed) {
    System.out.println("Turret Move: " + speed);
    m_turretMotor.set(speed);
  }

  public void stopTurret() {
    m_turretMotor.stopMotor();
  }

  public Command turretManualCommand() {
    return Commands.startEnd(
        () -> moveTurret(TurretConstants.turretSpeed),
        this::stopTurret,
        this
    );
  }

  public Command turretMoveManualCommand() {
    return Commands.runOnce(() -> moveTurret(TurretConstants.turretSpeed), this);
  }

  public Command turretStopManualCommand() {
    return Commands.runOnce(this::stopTurret, this);
  }

  @Override
  public void periodic() {
    // called once per scheduler run (about every 20ms)
    showEncoderPositions();
  }
}
