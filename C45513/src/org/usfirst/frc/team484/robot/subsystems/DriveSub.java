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
		setDefaultCommand(new JoystickDrive());
	}

	/**
	 * Tells the drivetrain to disable motion
	 */
	public static void doNothing() {
		if (RobotIO.drive == null) return;
		RobotIO.drive.tankDrive(0, 0);
	}
	
	/**
	 * Arcade drive with squared inputs
	 * @param y - forward power
	 * @param rot - rotation power
	 */

	public static void squaredDrive(double y, double rot) {
		if (RobotIO.drive == null) return;
		RobotIO.drive.arcadeDrive(y, rot, true);
	}
	
	/**
	 * Drives with the rotation value multiplied by the y value to reduce current draw
	 * and increase control at higher speeds.
	 * @param y - forward power
	 * @param rot - rotation power
	 */
	public static void curvatureDrive(double y, double rot) {
		if (RobotIO.drive == null) return;
		RobotIO.drive.curvatureDrive(y, rot, false);
	}

	/**
	 * Arcade drive with linear inputs
	 * @param y - forward power
	 * @param rot - rotation power
	 */
	public static void linearDrive(double y, double rot) {
		if (RobotIO.drive == null) return;
		RobotIO.drive.arcadeDrive(y, rot, false);
	}
	
	
	public static double pidOut1, pidOut2;
	public static void doublePIDDrive() {
		linearDrive(pidOut1, pidOut2);
	}

	/**
	 * zeros drivetrain encoders
	 */
	public static void resetEncoders() {
		RobotIO.leftEncoder.reset();
		RobotIO.rightEncoder.reset();
		
	}

	/**
	 * Gets the current heading (yaw) of the gyro.
	 * @return - yaw in degrees
	 */
    public static double getHeading() {
    	double[] ypr = new double[3];
    	RobotIO.imu.getYawPitchRoll(ypr);
    	return ypr[0];
    }

    /**
     * Drives each side of the robot using independent power outputs.
     * @param left [-1 to 1] % power to left side of robot
     * @param right [-1 to 1] % power to right side of robot
     */
    public static void tankDrive(double left, double right) {
    	RobotIO.drive.tankDrive(left, right, false);
    }
}

