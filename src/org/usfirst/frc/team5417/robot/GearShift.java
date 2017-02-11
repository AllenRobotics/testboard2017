package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;

public class GearShift {

	public static class GearSelection {
		public final int mode;
		
		public GearSelection(int mode) {
			this.mode = mode;
		}
	}
	public static final GearSelection kLowGear = new GearSelection(2);
	public static final GearSelection kHighGear = new GearSelection(3);
	
	private GearSelection currentGear = kLowGear;
	
	DoubleSolenoid shiftSolenoid1;
	DoubleSolenoid shiftSolenoid2;
	
	public GearShift(DoubleSolenoid shiftSolenoid1, DoubleSolenoid shiftSolenoid2) {
		this.shiftSolenoid1 = shiftSolenoid1;
		this.shiftSolenoid2 = shiftSolenoid2;
	}

	public GearSelection getCurrentGear() {
		return currentGear;
	}

	public void initGearShift() {
		shiftSolenoid1.set(Value.kReverse);
		shiftSolenoid2.set(Value.kReverse);
		SmartDashboard.putString("GearShift", "LowGear");
	}
	
	public boolean gearSwitch(final double leftMotorPower, final double rightMotorPower) {
		// don't switch if the motors are moving (safety)
		if (leftMotorPower == 0 && rightMotorPower == 0) {
			// switch to high gear if we're in low gear right now
			if (this.getCurrentGear() == kLowGear) { 
				shiftSolenoid1.set(Value.kForward);
				shiftSolenoid2.set(Value.kForward);
				currentGear = kHighGear;
			}
			// otherwise, switch to low gear if we're in high gear right now
			else {
				shiftSolenoid1.set(Value.kReverse);
				shiftSolenoid2.set(Value.kReverse);
				currentGear = kLowGear;
			}

			return true;
		}

		else return false;
	}
}
