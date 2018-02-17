package org.usfirst.frc.team484.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotIO {
	private static List<WPI_TalonSRX> leftTalons = new ArrayList<>();
	private static List<WPI_TalonSRX> rightTalons = new ArrayList<>();

	public static SpeedControllerGroup leftDriveMotors;
	public static SpeedControllerGroup rightDriveMotors;
	public static VictorSP elevatorMotor;
	public static VictorSP grabberAngleMotor;
	public static VictorSP leftClimberMotor;
	public static VictorSP rightClimberMotor;

	public static DifferentialDrive drive;

	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	public static Encoder elevatorEncoder;

	public static DigitalInput grabberAngleUpDI;
	public static DigitalInput grabberAngleDownDI;

	public static DoubleSolenoid shifterSolenoid;
	public static DoubleSolenoid grabberSolenoid;

	public static Joystick driveStick;
	public static Joystick opStick;

	public static PigeonIMU imu;

	public static RobotLogger logger;

	public static PowerDistributionPanel pdp;

	/**
	 * Initializes all objects connected to the RoboRIO.
	 */
	public RobotIO() {

		//-----Initialize all rotary actuators-----
		for (int id : RobotSettings.LEFT_DRIVE_MOTOR_IDS) {
			try {
				WPI_TalonSRX talon = new WPI_TalonSRX(id);
				leftTalons.add(talon);
				talon.setName("Left Drive", "Motor (" + id + ")");
				talon.setInverted(RobotSettings.INVERT_LEFT_MOTORS);
				if (id == RobotSettings.IMU_TALON_ID && imu == null) {
					imu = new PigeonIMU(talon);
				}
			} catch (Throwable t) {
				t.printStackTrace();
			}
		}
		for (int id : RobotSettings.RIGHT_DRIVE_MOTOR_IDS) {
			try {
				WPI_TalonSRX talon = new WPI_TalonSRX(id);
				rightTalons.add(talon);
				talon.setName("Right Drive", "Motor (" + id + ")");
				talon.setInverted(RobotSettings.INVERT_RIGHT_MOTORS);
				if (id == RobotSettings.IMU_TALON_ID && imu == null) {
					imu = new PigeonIMU(talon);
				}
			} catch (Throwable t) {
				t.printStackTrace();
			}
		}
		try {
			leftDriveMotors = new SpeedControllerGroup(leftTalons.get(0),
					leftTalons.subList(1, leftTalons.size()).toArray(new SpeedController[leftTalons.size() - 1]));
			rightDriveMotors = new SpeedControllerGroup(rightTalons.get(0),
					rightTalons.subList(1, rightTalons.size()).toArray(new SpeedController[rightTalons.size() - 1]));
			drive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			elevatorMotor = new VictorSP(RobotSettings.ELEVATOR_MOTOR_PORT);
			elevatorMotor.setName("Elevator", "Motor");
			elevatorMotor.setInverted(RobotSettings.INVERT_ELEVATOR_MOTOR);
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			grabberAngleMotor = new VictorSP(RobotSettings.GRABBER_ANGLE_MOTOR_PORT);
			grabberAngleMotor.setName("Grabber", "Angle Motor");
			grabberAngleMotor.setInverted(RobotSettings.INVERT_GRABBER_ANGLE_MOTOR);
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			leftClimberMotor = new VictorSP(RobotSettings.LEFT_CLIMBER_MOTOR_PORT);
			leftClimberMotor.setName("Climber", "Left Motor");
			leftClimberMotor.setInverted(RobotSettings.INVERT_LEFT_CLIMBER_MOTOR);
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			rightClimberMotor = new VictorSP(RobotSettings.RIGHT_CLIMBER_MOTOR_PORT);
			rightClimberMotor.setName("Climber", "Right Motor");
			rightClimberMotor.setInverted(RobotSettings.INVERT_RIGHT_CLIMBER_MOTOR);
		} catch (Throwable t) {
			t.printStackTrace();
		}


		//-----Initialize all rotary encoders-----
		try {
			leftEncoder = new Encoder(RobotSettings.LEFT_ENCODER_A_CHANNEL, RobotSettings.LEFT_ENCODER_B_CHANNEL);
			leftEncoder.setDistancePerPulse(RobotSettings.LEFT_ENCODER_DISTANCE_PER_PULSE);
			leftEncoder.setName("Left Drive", "Encoder");
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			rightEncoder = new Encoder(RobotSettings.RIGHT_ENCODER_A_CHANNEL, RobotSettings.RIGHT_ENCODER_B_CHANNEL);
			rightEncoder.setDistancePerPulse(RobotSettings.RIGHT_ENCODER_DISTANCE_PER_PULSE);
			rightEncoder.setName("Right Drive", "Encoder");
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			elevatorEncoder = new Encoder(RobotSettings.ELEVATOR_ENCODER_A_CHANNEL, RobotSettings.ELEVATOR_ENCODER_B_CHANNEL);
			elevatorEncoder.setDistancePerPulse(RobotSettings.ELEVATOR_ENCODER_DISTANCE_PER_PULSE);
			elevatorEncoder.setName("Elevator", "Encoder");
		} catch (Throwable t) {
			t.printStackTrace();
		}


		//-----Initialize remaining DIO-----
		try {
			grabberAngleUpDI = new DigitalInput(RobotSettings.GRABBER_ANGLE_UP_DI_PORT);
			grabberAngleUpDI.setName("Grabber", "Up Switch");
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			grabberAngleDownDI = new DigitalInput(RobotSettings.GRABBER_ANGLE_DOWN_DI_PORT);
			grabberAngleDownDI.setName("Grabber", "Down Switch");
		} catch (Throwable t) {
			t.printStackTrace();
		}

		//-----Initialize all solenoid actuators-----
		try {
			shifterSolenoid = new DoubleSolenoid(RobotSettings.SHIFTER_SOLENOID_HIGH_GEAR_PORT,
					RobotSettings.SHIFTER_SOLENOID_LOW_GEAR_PORT);
			shifterSolenoid.setName("Shifter", "Solenoid");
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			grabberSolenoid = new DoubleSolenoid(RobotSettings.GRABBER_CLOSE_PORT, RobotSettings.GRABBER_OPEN_PORT);
			grabberSolenoid.setName("Grabber", "Solenoid");
		} catch (Throwable t) {
			t.printStackTrace();
		}

		//-----Initialize all HIDs-----
		try {
			driveStick = new Joystick(RobotSettings.DRIVE_STICK_PORT);
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			opStick = new Joystick(RobotSettings.OP_STICK_PORT);
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			pdp = new PowerDistributionPanel();
		} catch (Throwable t) {
			t.printStackTrace();
		}
		try {
			SmartDashboard.putData(pdp);
			//-----Creates logger-----
			logger = new RobotLogger(RobotSettings.LOGGER_UPDATE_RATE);
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	/**
	 * Toggles voltage compensation in the TalonSRXs used in the drivetrain. This is useful for 
	 * PID autonomous operation.
	 */
	public static void setVoltageComp(boolean enabled) {
		try {
			for (WPI_TalonSRX talon : leftTalons) {
				talon.configVoltageCompSaturation(RobotSettings.VOLTAGE_TARGET, RobotSettings.CAN_COMMAND_TIMEOUT);
				talon.enableVoltageCompensation(enabled);
			}
			for (WPI_TalonSRX talon : rightTalons) {
				talon.configVoltageCompSaturation(RobotSettings.VOLTAGE_TARGET, RobotSettings.CAN_COMMAND_TIMEOUT);
				talon.enableVoltageCompensation(enabled);
			}
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	/**
	 * Fuses the encoder distances from both encoders. This method will only return the distance of
	 * 1 encoder if the 2nd encoder is deemed inoperable.
	 */
	public static double getFusedEncoderDistance() {
		if (leftEncoder == null && rightEncoder != null) return rightEncoder.getDistance();
		if (rightEncoder == null && leftEncoder != null) return leftEncoder.getDistance();
		if (leftEncoder == null && rightEncoder == null) return 0;
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
		if (leftEncoder == null && rightEncoder != null) return rightEncoder.getRate();
		if (rightEncoder == null && leftEncoder != null) return leftEncoder.getRate();
		if (leftEncoder == null && rightEncoder == null) return 0;
		if (Math.abs(leftEncoder.getRate()) < 1) {
			return rightEncoder.getRate();
		} 
		if (Math.abs(rightEncoder.getRaw()) < 1) {
			return leftEncoder.getRate();
		}
		return (leftEncoder.getRate() + rightEncoder.getRate()) / 2.0;
	}
}
