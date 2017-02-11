package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	//private CameraServer cameraServer;
	// XboxController stick = new XboxController(0);
	// XboxController stick1 = new XboxController(1);
	CANTalon leftFrontMotor = new CANTalon(0);
	CANTalon leftRearMotor = new CANTalon(1);
	CANTalon rightFrontMotor = new CANTalon(2);
	CANTalon rightRearMotor = new CANTalon(3);
	Spark shooterMotor1 = new Spark(0);
	Spark shooterMotor2 = new Spark(1);
	Spark intakeMotor = new Spark(2);
	Spark climberMotor1 = new Spark(3);
	Spark climberMotor2 = new Spark(4);
	Joystick manipulatorStick = new Joystick(1);
//	Solenoid gearSolenoid = new Solenoid(0,0);
	//DoubleSolenoid gearSolenoid = new DoubleSolenoid(0,1);
//	Solenoid shooterSolenoid1 = new Solenoid(0, 1);
//	Solenoid shooterSolenoid2 = new Solenoid(0,2);
	Compressor compressor = new Compressor(0);
	GearShift driveSystem;
	DoubleSolenoid shiftSolenoid1 = new DoubleSolenoid(0,1);
	DoubleSolenoid shiftSolenoid2 = new DoubleSolenoid(2,3);
	RobotDrive myRobot = new RobotDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
	boolean wasAPressed = false;
	boolean buttonPress = false;
	XBoxController driverStick = new XBoxController(new Joystick(0));
//	TickUpdate ticks = new TickUpdate();

	final String defaultAuto = "Default Autonomous";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();

	public Robot() {
		myRobot.setExpiration(0.1);
	}

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
		compressor.start();
		driveSystem = new GearShift(shiftSolenoid1, shiftSolenoid2);
//		ticks.reset();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomous() {
		String autoSelected = chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		switch (autoSelected) {
		case customAuto:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 1.0); // spin at half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		case defaultAuto:
		default:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		}
	}
	/*
	 * Runs the motors with tank steering.
	 */
	/*
	@Override*/
	/*protected void disabled() {
		if (isInitialized) {
			doOneTimeShutdown();
			isInitialized = false;
		}
	}*/
	
	public double getJoystickValueWithDeadZone(double value) {
		double deadZone = 0.1;
		
		if (value > -deadZone && value < deadZone) {
			value = 0;
		}
		return value;
	}
	
	@Override
	public void operatorControl() {
		
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			
			double leftY = getJoystickValueWithDeadZone(driverStick.getLYValue());
			double rightY = getJoystickValueWithDeadZone(driverStick.getRYValue());
			
			// myRobot.tankDrive(stick.getY(Hand.kLeft),
			// stick.getY(Hand.kRight)); // drive with tank style (use right +
			// left stick)
			// myRobot.TankDrive(stick.GetRawAxis(<left_y_axis>),
			// stick.GetRawAxis(<right_y_axis>));
			myRobot.tankDrive(driverStick.getLYValue(), driverStick.getRYValue());
			Timer.delay(0.005); // wait for a motor update time
			
			// TODO: Need to figure out if we want this piston extended or retracted


				if(driverStick.isFirstLBPressed()){
					// for use with the real robot
					//boolean didSwitchGears = driveSystem.gearSwitch(leftFrontMotor.get(), rightFrontMotor.get());

					// FOR TESTING ONLY - DO NOT USE WITH THE REAL ROBOT
					boolean didSwitchGears = driveSystem.gearSwitch(leftY, rightY);
					
					
					SmartDashboard.putBoolean("DidSwitchGears", didSwitchGears);
				}
				
				if (driveSystem.getCurrentGear() == GearShift.kLowGear) {
					SmartDashboard.putString("GearShift", "LowGear");
				}
				else {
					SmartDashboard.putString("GearShift", "HighGear");
				}
			
/*			
			if (driverStick.getRawButton(1) == true && gearSolenoid.get() == Value.kReverse) //detects if the A button is pressed 
				gearSolenoid.set(DoubleSolenoid.Value.kForward); // extends gear piston
			else if (!driverStick.getRawButton(1) && gearSolenoid.get() == Value.kForward)
				gearSolenoid.set(DoubleSolenoid.Value.kReverse);
	
			if (manipulatorStick.getRawAxis(2) >=.5) { // detects if left trigger is pressed more than halfway
				shooterSolenoid1.set(false); // retracts piston, lets balls enter shooter
				shooterMotor1.set(-1); // activates shooter motor
			}.
			else {
				shooterSolenoid1.set(true); 
				shooterMotor1.set(0);
			}
			if (manipulatorStick.getRawAxis(3) >=.5) {
				shooterSolenoid2.set(false);
				shooterMotor2.set(1);
			}
			else {
				shooterSolenoid2.set(true);
				shooterMotor2.set(0);
			}
*/
			if (driverStick.getRTValue() >= .5)
				intakeMotor.set(1);
			else if (driverStick.isRBHeldDown()) 
				intakeMotor.set(-1);
			else intakeMotor.set(0);
			
			if (manipulatorStick.getRawButton(1)){
				climberMotor1.set(1); climberMotor2.set(1);
			}
			
			else{
			climberMotor1.set(0);
			climberMotor2.set(0);
			
			}
			
			boolean switchPressure = compressor.getPressureSwitchValue();
			boolean compressorState = compressor.enabled();
			SmartDashboard.putString("DB/String 0", "compressor state:" + compressorState);
			SmartDashboard.putString("DB/String 1", "Pressure switch:" + switchPressure);
		}
	}
	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}
}
