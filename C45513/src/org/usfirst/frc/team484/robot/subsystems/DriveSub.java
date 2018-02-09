package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.commands.JoystickDrive;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for controlling the drive train motors.
 */
public class DriveSub extends Subsystem {

	/**
	 * Sets the command that runs for this subsystem when no
	 * other commands are running that require the subsystem.
	 */
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new JoystickDrive());
	}

	/**
	 * Tells the drivetrain to disable motion
	 */
	public static void doNothing() {
		RobotIO.drive.tankDrive(0, 0);
	}
	/**
	 * Arcade drive with squared inputs
	 * @param y - forward power
	 * @param rot - rotation power
	 */

	public static void squaredDrive(double y, double rot) {
		RobotIO.drive.arcadeDrive(y, rot, true);
	}

	/**
	 * Arcade drive with linear inputs
	 * @param y - forward power
	 * @param rot - rotation power
	 */
	public static void linearDrive(double y, double rot) {
		RobotIO.drive.arcadeDrive(y, rot, false);
	}
}

