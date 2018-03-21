package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.JoystickElevator;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for controlling the cube elevator.
 */
public class ElevatorSub extends Subsystem {
	
	/**
	 * Sets the command that runs for this subsystem when no
	 * other commands are running that require the subsystem.
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new JoystickElevator());
	}

	/**
	 * Applies an output of 0 to the elevator motor.
	 */
	public static void doNothing() {
		applyPower(0);
	}

	/**
	 * Sets the output power of the elevator motor.
	 * @param speed - the power output from -1 to 1.
	 */
	public static void setRate(double speed) {
		if (speed > 0 && isUp()) {
			applyPower(RobotSettings.ELEVATOR_GRAVITY_COMPENSATION_POWER);
		} else if (speed < 0 && isDown()) {
			applyPower(RobotSettings.ELEVATOR_GRAVITY_COMPENSATION_POWER / 2.0);
		} else {
			applyPower(speed + RobotSettings.ELEVATOR_GRAVITY_COMPENSATION_POWER);
		}
	}

	/**
	 * Returns true if the elevator is all the way down using a limit switch.
	 * Will return false if the switch is unplugged or cannot be instantiated.
	 * @return true if elevator is down
	 */
	public static boolean isDown() {
		if (RobotIO.elevatorDownDI == null) return false;
		return !RobotIO.elevatorDownDI.get();
	}

	/**
	 * Returns true if the elevator is all the way up using the hall-effect.
	 * Will return false if the sensor is unplugged or cannot be instantiated.
	 * @return true if elevator is up
	 */
	public static boolean isUp() {
		if (RobotIO.elevatorUpDI == null) return false;
		SmartDashboard.putBoolean("is up", RobotIO.elevatorUpDI.get());
		return !RobotIO.elevatorUpDI.get();
	}

	private static void applyPower(double output) {
		RobotIO.elevatorMotorL1.set(-output);
		RobotIO.elevatorMotorL2.set(-output);
		RobotIO.elevatorMotorR1.set(-output);
		RobotIO.elevatorMotorR2.set(-output);
	}

}

