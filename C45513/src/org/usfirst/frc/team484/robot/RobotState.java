package org.usfirst.frc.team484.robot;

import java.io.Serializable;

import org.usfirst.frc.team484.robot.subsystems.DriveSub;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;
import org.usfirst.frc.team484.robot.subsystems.GrabberAngleSub;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * This class serves to record the state of the robot at a given moment in time
 * State information includes not just the position and power output of robot
 * components, but also their speed. The state is recorded to this struct upon
 * initialization.
 *
 */
public class RobotState implements Serializable {
	private static final long serialVersionUID = 1L;

	//----- DriveTrain variables-----
	public final double leftPwr;
	public final double rightPwr;
	public final double leftDist;
	public final double rightDist;
	public final double leftVel;
	public final double rightVel;
	public final boolean shifterState;
	public final double robotAngle;

	//----- Elevator variables -----
	public final double elevPwr;
	public final double elevPos;
	public final double elevVel;
	public final boolean elevatorDown;
	public final boolean elevatorUp;

	//----- Grabber variables -----
	public final boolean grabberState;
	public final boolean grabberDown;
	public final boolean grabberUp;
	public final double grabberPower;
	public RobotState() {
		//----- DriveTrain variables-----
		leftPwr = RobotIO.leftDriveMotors.get();
		rightPwr = RobotIO.rightDriveMotors.get();
		leftDist = RobotIO.leftEncoder.getDistance();
		rightDist = RobotIO.rightEncoder.getDistance();
		leftVel = RobotIO.leftEncoder.getRate();
		rightVel = RobotIO.rightEncoder.getRate();
		shifterState = RobotIO.shifterSolenoid.get().equals(Value.kForward);
		robotAngle = DriveSub.getHeading();

		//----- Elevator variables -----
		elevPwr = ElevatorSub.getPower();
		elevPos = RobotIO.getElevatorHeight();
		elevVel = RobotIO.getElevatorRate();
		elevatorDown = ElevatorSub.isDown();
		elevatorUp = ElevatorSub.isUp();

		//----- Grabber variables -----
		grabberState = RobotIO.grabberSolenoid.get().equals(Value.kForward);
		grabberDown = GrabberAngleSub.isDown();
		grabberUp = GrabberAngleSub.isUp();
		grabberPower = RobotIO.grabberAngleMotor.get();
	}
}
