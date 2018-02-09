package org.usfirst.frc.team484.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotIO {
	private static List<WPI_TalonSRX> leftTalons = new ArrayList<>();
	private static List<WPI_TalonSRX> rightTalons = new ArrayList<>();
	
	public static SpeedControllerGroup leftDriveMotors;
	public static SpeedControllerGroup rightDriveMotors;
	public static VictorSP elevatorMotor;
	
	public static DifferentialDrive drive;
	
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	public static Encoder elevatorEncoder;
	
	public static DoubleSolenoid shifterSolenoid;
	public static DoubleSolenoid grabberSolenoid;
	
	public static Joystick driveStick;
	public static Joystick opStick;
	
	public static PigeonIMU imu;
	
	public static RobotLogger logger;
	
	/**
	 * Initializes all objects connected to the RoboRIO.
	 */
	public RobotIO() {
		
		//-----Initialize all rotary actuators-----
		for (int id : RobotSettings.LEFT_DRIVE_MOTOR_IDS) {
			WPI_TalonSRX talon = new WPI_TalonSRX(id);
			leftTalons.add(talon);
			talon.setName("Left Drive", "Motor (" + id + ")");
			talon.setInverted(RobotSettings.INVERT_LEFT_MOTORS);
			if (id == RobotSettings.IMU_TALON_ID && imu == null) {
				imu = new PigeonIMU(talon);
			}
		}
		for (int id : RobotSettings.RIGHT_DRIVE_MOTOR_IDS) {
			WPI_TalonSRX talon = new WPI_TalonSRX(id);
			rightTalons.add(talon);
			talon.setName("Right Drive", "Motor (" + id + ")");
			talon.setInverted(RobotSettings.INVERT_RIGHT_MOTORS);
			if (id == RobotSettings.IMU_TALON_ID && imu == null) {
				imu = new PigeonIMU(talon);
			}
		}
		
		leftDriveMotors = new SpeedControllerGroup(leftTalons.get(0),
				leftTalons.subList(1, leftTalons.size()).toArray(new SpeedController[leftTalons.size() - 1]));
		rightDriveMotors = new SpeedControllerGroup(rightTalons.get(0),
				rightTalons.subList(1, rightTalons.size()).toArray(new SpeedController[rightTalons.size() - 1]));

		elevatorMotor = new VictorSP(RobotSettings.ELEVATOR_MOTOR_PORT);
		elevatorMotor.setName("Elevator", "Motor");
		
		drive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
		
		
		//-----Initialize all rotary encoders-----
		leftEncoder = new Encoder(RobotSettings.LEFT_ENCODER_A_CHANNEL, RobotSettings.LEFT_ENCODER_B_CHANNEL);
		rightEncoder = new Encoder(RobotSettings.RIGHT_ENCODER_A_CHANNEL, RobotSettings.RIGHT_ENCODER_B_CHANNEL);
		elevatorEncoder = new Encoder(RobotSettings.ELEVATOR_ENCODER_A_CHANNEL, RobotSettings.ELEVATOR_ENCODER_B_CHANNEL);
		
		leftEncoder.setDistancePerPulse(RobotSettings.LEFT_ENCODER_DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(RobotSettings.RIGHT_ENCODER_DISTANCE_PER_PULSE);
		elevatorEncoder.setDistancePerPulse(RobotSettings.ELEVATOR_ENCODER_DISTANCE_PER_PULSE);
		
		leftEncoder.setName("Left Drive", "Encoder");
		rightEncoder.setName("Right Drive", "Encoder");
		elevatorEncoder.setName("Elevator", "Encoder");
		
		//-----Initialize all solenoid actuators-----
		shifterSolenoid = new DoubleSolenoid(RobotSettings.SHIFTER_SOLENOID_HIGH_GEAR_PORT,
				RobotSettings.SHIFTER_SOLENOID_LOW_GEAR_PORT);
		grabberSolenoid = new DoubleSolenoid(RobotSettings.GRABBER_CLOSE_PORT, RobotSettings.GRABBER_OPEN_PORT);
		shifterSolenoid.setName("Shifter", "Solenoid");
		grabberSolenoid.setName("Grabber", "Solenoid");
		
		//-----Initialize all HIDs-----
		driveStick = new Joystick(RobotSettings.DRIVE_STICK_PORT);
		opStick = new Joystick(RobotSettings.OP_STICK_PORT);
		
		//-----Creates logger-----
		logger = new RobotLogger(RobotSettings.LOGGER_UPDATE_RATE);
	}
	
	/**
	 * Toggles voltage compensation in the TalonSRXs used in the drivetrain. This is useful for 
	 * PID autonomous operation.
	 */
	public static void setVoltageComp(boolean enabled) {
		for (WPI_TalonSRX talon : leftTalons) {
			talon.configVoltageCompSaturation(RobotSettings.VOLTAGE_TARGET, RobotSettings.CAN_COMMAND_TIMEOUT);
			talon.enableVoltageCompensation(enabled);
		}
		for (WPI_TalonSRX talon : rightTalons) {
			talon.configVoltageCompSaturation(RobotSettings.VOLTAGE_TARGET, RobotSettings.CAN_COMMAND_TIMEOUT);
			talon.enableVoltageCompensation(enabled);
		}
	}
	
	/**
	 * Fuses the encoder distances from both encoders. This method will only return the distance of
	 * 1 encoder if the 2nd encoder is deemed inoperable.
	 */
	public static double getFusedEncoderDistance() {
		if (Math.abs(leftEncoder.getRaw()) < 10 ||
				Math.abs(leftEncoder.getRaw()) * 2.0 < Math.abs(rightEncoder.getRaw())) {
			return rightEncoder.getDistance();
		} 
		if (Math.abs(rightEncoder.getRaw()) < 10 ||
				Math.abs(rightEncoder.getRaw()) * 2.0 < Math.abs(leftEncoder.getRaw())) {
			return leftEncoder.getDistance();
		}
		return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
	}
	
	/**
	 * Fuses the encoder rates from both encoders. This method will only return the rate of
	 * 1 encoder if the 2nd encoder is deemed inoperable. It is suggested that this method only
	 * be used if the robot is traveling in a straight line.
	 */
	public static double getFusedEncoderRate() {
		if (Math.abs(leftEncoder.getRate()) < 1) {
			return rightEncoder.getRate();
		} 
		if (Math.abs(rightEncoder.getRaw()) < 1) {
			return leftEncoder.getRate();
		}
		return (leftEncoder.getRate() + rightEncoder.getRate()) / 2.0;
	}
}
