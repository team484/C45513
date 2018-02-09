package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.commands.GrabberAngleDoNothing;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
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
	
	public static boolean isDown() {
		return RobotIO.grabberAngleDownDI.get();
	}
	
	public static boolean isUp() {
		return RobotIO.grabberAngleUpDI.get();
	}
}

