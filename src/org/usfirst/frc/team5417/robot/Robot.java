package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5417.cv2017.Stopwatch;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

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
public class Robot extends SampleRobot implements PIDOutput {
	// private CameraServer cameraServer;
	// XboxController stick = new XboxController(0);
	// XboxController stick1 = new XboxController(1);
	CANTalon leftFrontMotor = new CANTalon(2);
	CANTalon leftRearMotor = new CANTalon(3);
	CANTalon rightFrontMotor = new CANTalon(4);
	CANTalon rightRearMotor = new CANTalon(5);
	CANTalon leftShooterMotor = new CANTalon(6);
	CANTalon rightShooterMotor = new CANTalon(7);
	CANTalon intakeMotor = new CANTalon(8);
	CANTalon climberMotor1 = new CANTalon(9);
	CANTalon climberMotor2 = new CANTalon(10);
	Solenoid gearSolenoid = new Solenoid(1, 0);
	// DoubleSolenoid gearSolenoid = new DoubleSolenoid(0,1);
	Solenoid shooterSolenoid1 = new Solenoid(1, 1);
	Solenoid shooterSolenoid2 = new Solenoid(1, 2);
	Compressor compressor = new Compressor(1);
	GearShift driveSystem;
	Solenoid shiftSolenoid1 = new Solenoid(0, 0);
	Solenoid shiftSolenoid2 = new Solenoid(0, 1);
	RobotDrive myRobot = new RobotDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
	XBoxController driverStick = new XBoxController(new Joystick(0));
	XBoxController manipulatorStick = new XBoxController(new Joystick(0));
	AHRS ahrs;
	PIDController turnController;
	static final double kP = .1;
	static final double kI = .001;
	static final double kD = 0.00;
	static final double kF = 0.00;

	static final double kToleranceDegrees = 2;
	double rotateToAngleRate = 0;

	Stopwatch elapsedTimeWithNoDriveInputs = null;

	final String defaultAuto = "Default Autonomous";
	final String customAuto = "My Auto";
	final String otherAuto = "other auto";
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

		ahrs = new AHRS(I2C.Port.kOnboard);
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-0.5, 0.5);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
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
	 * @Override
	 */
	/*
	 * protected void disabled() { if (isInitialized) { doOneTimeShutdown();
	 * isInitialized = false; } }
	 */

	public double getJoystickValueWithDeadZone(double value) {
		double deadZone = 0.2;

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

			if (leftY == 0 && rightY == 0) {
				if (elapsedTimeWithNoDriveInputs == null) {
					elapsedTimeWithNoDriveInputs = Stopwatch.startNew();
				}
			} else {
				elapsedTimeWithNoDriveInputs = null;
			}

			// apply rotateToAngleRate if necessary
			if (turnController.isEnabled()) {
				leftY = leftY / 2 + rotateToAngleRate;
				rightY = rightY / 2 - rotateToAngleRate;
			}
			myRobot.tankDrive(leftY, rightY);

			Timer.delay(0.005); // wait for a motor update time

			// TODO: Need to figure out if we want this piston extended or
			// retracted

			// ///////////////////////////////////////////////////////
			// BEGIN BUTTON IF ELSE SECTION
			//
			//

			// gear shift section
			if (driverStick.isFirstLBPressed() && getElapsedSecondsWithNoDriveInputs() > 0.1) {
				driveSystem.gearShiftLow(leftY, rightY);

			} else if (driverStick.isFirstRBPressed() && getElapsedSecondsWithNoDriveInputs() > 0.1) {
				driveSystem.gearShiftHigh(leftY, rightY);
			}
			// navX control section
			else if (driverStick.isStartHeldDown()) {
				ahrs.reset();
				turnController.setSetpoint(ahrs.getAngle());
			}

			// Drive straight enable/disable
			if (driverStick.isDPadUpHeldDown()) {
				turnController.enable();
			} else {
				turnController.disable();
			}

			// intake section
			if (driverStick.getRTValue() >= .5)
				intakeMotor.set(1);
			else if (driverStick.getLTValue() >= 5)
				intakeMotor.set(-1);
			else
				intakeMotor.set(0);

			// climber section
			if (manipulatorStick.isAHeldDown()) {
				climberMotor1.set(1);
				climberMotor2.set(1);
			}

			else {
				climberMotor1.set(0);
				climberMotor2.set(0);

			}

			// TESTING

			if (driverStick.isAHeldDown() == true && gearSolenoid.get() == false) {
				// detects if the A button is pressed
				gearSolenoid.set(true);
			}
			// extends gear piston
			else if (!driverStick.isAHeldDown() && gearSolenoid.get() == true) {
				gearSolenoid.set(false);
			}

			if (manipulatorStick.getLTValue() >= .5) {
				// detects if left trigger is pressed more than halfway
				// shooterSolenoid1.set(false);
				// retracts piston, lets balls enter shooter
				leftShooterMotor.set(-1); // activates shooter motor }. else {
				shooterSolenoid1.set(true);
				leftShooterMotor.set(0);
			}
			if (manipulatorStick.getRTValue() >= .5) {
				shooterSolenoid2.set(false);
				rightShooterMotor.set(1);
			} else {
				shooterSolenoid2.set(true);
				rightShooterMotor.set(0);
			}

			//
			// END BUTTON IF ELSE SECTION
			// ///////////////////////////////////////////////////////

			if (driveSystem.getCurrentGear() == GearShift.kLowGear) {
				SmartDashboard.putString("GearShift", "LowGear");
			} else {
				SmartDashboard.putString("GearShift", "HighGear");
			}

			boolean switchPressure = compressor.getPressureSwitchValue();
			boolean compressorState = compressor.enabled();
			SmartDashboard.putString("DB/String 0", "compressor state:" + compressorState);
			SmartDashboard.putString("DB/String 1", "Pressure switch:" + switchPressure);

			SmartDashboard.putNumber("left speed", leftY);
			SmartDashboard.putNumber("Right speed", rightY);
			SmartDashboard.putString("Turn controller", turnController.isEnabled() ? "true" : "false");
			SmartDashboard.putNumber("rotation", ahrs.getAngle());

			SmartDashboard.putNumber("time w/o drive inputs", getElapsedSecondsWithNoDriveInputs());
		}
	}

	private double getElapsedSecondsWithNoDriveInputs() {
		return elapsedTimeWithNoDriveInputs == null ? 0 : elapsedTimeWithNoDriveInputs.getTotalSeconds();
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}

	@Override
	public void pidWrite(double output) {
		SmartDashboard.putNumber("Rotate to angle rate", output);
		rotateToAngleRate = output;
	}
}
