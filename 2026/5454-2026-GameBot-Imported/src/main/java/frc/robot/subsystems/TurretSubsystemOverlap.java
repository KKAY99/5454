package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.*;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import java.util.function.Supplier;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class TurretSubsystemOverlap extends SubsystemBase {
  private TalonFX m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private static final double GEAR_0_TOOTH_COUNT = 80.0;
  private static final double GEAR_1_TOOTH_COUNT = 10.0;
  private static final double GEAR_2_TOOTH_COUNT = 41.0;
  private static final double GEAR_3_TOOTH_COUNT = 40.0;

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

     System.out.println("Turret/SolveEnc1PhaseDeg", enc1);
   System.out.println("Turret/SolveEnc2PhaseDeg", enc2);
       System.out.println("Turret/SolveBestCost", bestCost);

    return bestTheta;
}
      
      System.out.println("Turret Move:" + speed);
    m_turretMotor.set(speed);
  }

  public void stopTurret(){
    m_turretMotor.stopMotor();
  }
  public Command turretManualCommand(){
      return Commands.startEnd(    ()->moveTurret(TurretConstants.turretSpeed),
                                          ()->stopTurret(),
                                          this);
  }

  public Command turretMoveManualCommand(){
    return Commands.runOnce(    ()->moveTurret(TurretConstants.turretSpeed),this);
  }
  public Command turretStopManualCommand(){
    return Commands.runOnce(    ()->stopTurret(),this);
  }
}
