package frc.robot.classes;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

 
public class SpectrumAxisButton extends Trigger {
		private final GenericHID joy;
		private final int axis;
		private double targetVal;
		private ThresholdType thresholdType;
		
	public static enum ThresholdType
	{
			LESS_THAN, GREATER_THAN, EXACT, POV, DEADBAND;	
	}

	public SpectrumAxisButton(Joystick joystick, int axis, double threshold, ThresholdType thresholdType) {
		super(BooleanSupplier -> get());//KK Added 
		this.joy = joystick;
		this.axis = axis;
		this.targetVal = threshold;
		this.thresholdType = thresholdType;
		super(this.get);
	}

    public SpectrumAxisButton(XboxController joystick, int axis, double threshold, ThresholdType thresholdType) {
		super((BooleanSupplier -> get()));
		this.joy = joystick;
		this.axis = axis;
		this.targetVal = threshold;
		this.thresholdType = thresholdType;
		
	
    }
    public double getAxis(int a){
		return -1;
		//Build this out so that if it's x or y or it flips it
	}
	

	public boolean get() {
		switch (this.thresholdType) {
		case EXACT:
			//System.out.println("axis value: " + joy.getRawAxis(this.axis));
			return joy.getRawAxis(this.axis) == this.targetVal;
		case LESS_THAN:
			return joy.getRawAxis(this.axis) < this.targetVal;
		case GREATER_THAN:
			return joy.getRawAxis(this.axis) > this.targetVal;
		case POV:
			return joy.getPOV() == this.targetVal;
		case DEADBAND:
			return Math.abs(joy.getRawAxis(this.axis)) > this.targetVal;
		default:
		return false;
		}
	}
	
}