package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;

public class GearShift {

	DoubleSolenoid shiftSolenoid1;
	DoubleSolenoid shiftSolenoid2;
	
	public GearShift(DoubleSolenoid shiftSolenoid1, DoubleSolenoid shiftSolenoid2) {
		this.shiftSolenoid1 = shiftSolenoid1;
		this.shiftSolenoid2 = shiftSolenoid2;
	}
	
	public void initGearShift() {
		shiftSolenoid1.set(Value.kReverse);
		shiftSolenoid2.set(Value.kReverse);
	}
	
	public void gearSwitch(final double leftMotorPower, final double rightMotorPower) {
		if (leftMotorPower == 0 && rightMotorPower == 0) {
			if (shiftSolenoid1.get() == Value.kReverse) 
				shiftSolenoid1.set(Value.kForward);
			else shiftSolenoid1.set(Value.kReverse);
			
			if (shiftSolenoid2.get() == Value.kReverse) 
				shiftSolenoid2.set(Value.kForward);
			else shiftSolenoid2.set(Value.kReverse);
			
			
		}
	}
}
