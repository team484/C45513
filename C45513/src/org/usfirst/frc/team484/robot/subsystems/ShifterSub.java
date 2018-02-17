package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for controlling the gear of the gearbox. (high or low gear)
 */
public class ShifterSub extends Subsystem {

	/**
	 * This subsystem has no default command.
	 */
	public void initDefaultCommand() {}

	/**
	 * Shifts the robot into high gear.
	 */
	public static void shiftHigh() {
		if (RobotIO.shifterSolenoid == null) return;
		RobotIO.shifterSolenoid.set(Value.kForward);
	}

	/**
	 * Shifts the robot into low gear.
	 */
	public static void shiftLow() {
		if (RobotIO.shifterSolenoid == null) return;
		RobotIO.shifterSolenoid.set(Value.kReverse);
	}
}

