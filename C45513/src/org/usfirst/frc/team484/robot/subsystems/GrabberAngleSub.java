package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.commands.GrabberAngleDoNothing;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls the motor responsible for changing the rotation of the grabber arm.
 */
public class GrabberAngleSub extends Subsystem {
	
	/**
	 * Sets the default command for this subsystem to a command that keeps the motor speed at 0.
	 * This is to ensure that when a rotation command ends, the motor stops.
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new GrabberAngleDoNothing());
	}

	/**
	 * Sets the rate at which the grabber rotates up and down.
	 * Positive for up.
	 * @param speed - the rate
	 */
	public static void setRotateSpeed(double speed) {
		
		if (RobotIO.grabberAngleMotor == null) return;
		if (speed > 0 && isUp()) {
			RobotIO.grabberAngleMotor.set(0);
			return;
		}

		if (speed < 0 && isDown()) {
			RobotIO.grabberAngleMotor.set(0);
			return;
		}
		
		RobotIO.grabberAngleMotor.set(speed);
	}
	
	/**
	 * Checks if the arm is already down
	 * @return true if arm is rotated down all the way.
	 */
	public static boolean isDown() {
		if (RobotIO.grabberAngleDownDI == null) return false;
		return !RobotIO.grabberAngleDownDI.get();
	}
	
	/**
	 * Checks if the arm is already up. Will only work when arm is powered.
	 * @return true if arm is rotated up all the way.
	 */
	public static boolean isUp() {
		return RobotIO.irSensor.getAverageVoltage() > 2.4;
	}
	
}

