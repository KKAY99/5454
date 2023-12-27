package com.team1678.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Constants.ArmConstants;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.lib.logger.Log;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends ServoMotorSubsystem {

    private static Arm mInstance;

    private final StatorCurrentLimitConfiguration kScrapeCurrentLimit = new StatorCurrentLimitConfiguration(true, 2, 2,
            0.0);

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm(Constants.ArmConstants.kArmServoConstants);
        }
        return mInstance;
    }

    private Arm(final ServoMotorSubsystemConstants constants) {
        super(constants);
        mMaster.overrideSoftLimitsEnable(true);
        setWantNeutralBrake(false);
        zeroSensors();
    }

    // Homing refers to moving the arm into it's "zero" position. 
    private boolean mHoming = true;
    private boolean is_climb = false; // Disables homing
    private boolean is_scraping = false; // Lowers current

    private void setWantNeutralBrake(boolean brake) {
        NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
        mMaster.setNeutralMode(mode);
        for (TalonFX motor : mSlaves) {
            motor.setNeutralMode(mode);
        }
    }

    public void setWantHome(boolean home) {
        if (is_climb) {
            mHoming = false;
            return;
        }
        mMaster.overrideSoftLimitsEnable(!home);
        mHoming = home;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setSetpointMotionMagic(mConstants.kHomePosition);
            }

            @Override
            public void onLoop(double timestamp) {
                // constantly re-homing unless in open loop
                if (Util.epsilonEquals(getSetpointHomed(), mConstants.kHomePosition, 1.0)
                        && atHomingLocation()) {
                    setWantHome(true);
                } else if (mControlState != ControlState.OPEN_LOOP) {
                    setWantHome(false);
                }
            }

            @Override
            public void onStop(double timestamp) {
                setWantNeutralBrake(true);
            }
        });
    }

    public void updateCurrentLimits() {
        if (is_scraping) {
            mMaster.configStatorCurrentLimit(kScrapeCurrentLimit, Constants.kCANTimeoutMs);
        } else {
            mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                    mConstants.kEnableStatorCurrentLimit,
                    mConstants.kStatorContinuousCurrentLimit,
                    mConstants.kStatorPeakCurrentLimit,
                    mConstants.kStatorPeakCurrentDuration), Constants.kCANTimeoutMs);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.overrideSoftLimitsEnable(false);
        } else {
            mMaster.overrideSoftLimitsEnable(true);
        }

        if (mHoming) {
            setOpenLoop(-0.5 / mConstants.kMaxVoltage);
            if (mPeriodicIO.master_current > ArmConstants.kHomingCurrentThreshold) {
                zeroSensors();  
                setOpenLoop(0.0);
                mHoming = false;
            }
            super.writePeriodicOutputs();
        } else {
            super.writePeriodicOutputs();
        }
    }

    public Request armRequest(double angle, boolean waitForPosition) {
        return new Request() {
            @Override
            public void act() {
                setSetpointMotionMagic(angle);
                is_climb = false;
                is_scraping = false;
                updateCurrentLimits();
            }

            @Override
            public boolean isFinished() {
                return waitForPosition ? Util.epsilonEquals(mPeriodicIO.position_units, angle, 3.0) : true;
            }
        };
    }

    public Request climbRequest(double angle) {
        return new Request() {
            @Override
            public void act() {
                setSetpointMotionMagic(angle);
                is_climb = true;
                is_scraping = false;
                updateCurrentLimits();
            }

            @Override
            public boolean isFinished() {
                return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.5);
            }
        };
    }

    public Request scrapeRequest(double angle) {
        return new SequentialRequest(
            scrapeDropRequest(angle),
            scrapeHoldRequest(angle)
        );
    }
    

    private Request scrapeDropRequest(double angle) {
        return new Request() {
            @Override
            public void act() {
                setSetpointMotionMagic(angle);
                is_scraping = false;
                updateCurrentLimits();
            }

            @Override
            public boolean isFinished() {
                return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.5);
            }
        };
    }

    private Request scrapeHoldRequest(double angle) {
        return new Request() {
            @Override
            public void act() {
                is_scraping = true;
                updateCurrentLimits();
            }

            @Override
            public boolean isFinished() {
                return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.5);
            }
        };
    }

    public Request armWaitRequest(double angle) {
        return new Request() {
            @Override 
            public void act() {

            }

            @Override 
            public boolean isFinished() {
                return Util.epsilonEquals(mPeriodicIO.position_units, angle, 1.0);
            }
        };
    }

    @Override
    public boolean atHomingLocation() {
        return mPeriodicIO.position_units < mConstants.kHomePosition || Util.epsilonEquals(mPeriodicIO.position_units, mConstants.kHomePosition, 3.0);
    }

    @Override
    public synchronized void setSetpointMotionMagic(double units) {
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
        mPeriodicIO.feedforward = Constants.ArmConstants.kKg * Math.cos(Math.toRadians(getPosition())) / mConstants.kMaxVoltage;
        if (mControlState != ControlState.MOTION_MAGIC) {
            mMaster.selectProfileSlot(kMotionProfileSlot, 0);
            mControlState = ControlState.MOTION_MAGIC;
        }
    }

    @Log
    public double getArmAngleUnits(){
        return mPeriodicIO.position_units;
    }
    
    @Log
    public double getArmAngleTicks(){
        return mPeriodicIO.position_ticks;
    }

    @Log
    public double getArmSetpoints(){
        return getSetpointHomed();
    }
    
    @Log
    public double getArmDemand(){
        return mPeriodicIO.demand;
    }
    
    @Log
    public double getArmVelocity(){
        return mPeriodicIO.velocity_ticks_per_100ms;
    }
    
    @Log
    public double getArmVolts(){
        return mPeriodicIO.output_voltage;
    }
    
    @Log
    public double getArmCurrent(){
        return mPeriodicIO.master_current;
    }
    
    @Log
    public boolean getArmHoming(){
        return mHoming;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
    
    @Log
    public double getMainMotorBusVolts() {
        return mMaster.getBusVoltage();
    }

    @Log
    public double getFollowerMotorBusVolts() {
        return mSlaves[0].getBusVoltage();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + " Angle Units", mPeriodicIO.position_units);
        SmartDashboard.putNumber(mConstants.kName + " Angle Ticks", mPeriodicIO.position_ticks);
        SmartDashboard.putNumber(mConstants.kName + " Setpoint", getSetpointHomed());
        SmartDashboard.putNumber(mConstants.kName + " Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber(mConstants.kName + " Velocity", mMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber(mConstants.kName + " Follower Velocity", mSlaves[0].getSelectedSensorVelocity());
        SmartDashboard.putNumber(mConstants.kName + " Output Voltage", mMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber(mConstants.kName + " Follower Output Voltage", mSlaves[0].getMotorOutputVoltage());
        SmartDashboard.putNumber(mConstants.kName + " Current", mPeriodicIO.master_current);
        SmartDashboard.putNumber(mConstants.kName + " Supply Current", mMaster.getSupplyCurrent());
        SmartDashboard.putBoolean(mConstants.kName + " Homing", mHoming);
    }
}
