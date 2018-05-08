package org.usfirst.frc.team484.robot.commands;

import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.ArrayList;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.RobotState;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;
import org.usfirst.frc.team484.robot.subsystems.GrabberAngleSub;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Loads a routine file from storage and loops through it
 */
public class PlayRoutine extends Command {
	private static final double kPDrive = 0.04;
	private static final double kDDrive = 0.01;
	private static final double kPAngle = -0.01;

	private static final double kPElev = 3.0;
	private static final double kDElev = 0.5;

	private static final String SAVE_DIR = RobotSettings.SAVE_DIR_ROUTINES;
	FileInputStream fis;
	ObjectInputStream ois;
	ArrayList<RobotState> stateArray;
	String name;
	int arrayPosition = 0;

	/**
	 * Creates a PlayRoutine command that will execute the routine file with
	 * the given name.
	 * @param routineName - The name of the routine file to execute
	 */
	public PlayRoutine(String routineName) {
		name = routineName;
		requires(Robot.driveSub);
		requires(Robot.elevatorSub);
		requires(Robot.grabberAngleSub);
		requires(Robot.grabberSub);
		requires(Robot.shifterSub);
	}

	// Called just before this Command runs the first time
	@SuppressWarnings("unchecked")
	protected void initialize() {
		if (!new File(SAVE_DIR + name).exists()) {
			new File(SAVE_DIR + name).mkdirs();
		}
		try {
			fis = new FileInputStream(SAVE_DIR + "/" + name + ".rtn");
			ois = new ObjectInputStream(fis);
			stateArray = (ArrayList<RobotState>) ois.readObject();
			ois.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		RobotIO.setVoltageComp(true);
		RobotIO.leftEncoder.reset();
		RobotIO.rightEncoder.reset();
		RobotIO.imu.setYaw(0, 100);
		arrayPosition = 0;
	}
	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (stateArray.size() <= arrayPosition) {
			return;
		}
		RobotState state = stateArray.get(arrayPosition);
		arrayPosition++;

		//----- Solenoids -----	
		if (state.shifterState) {
			RobotIO.shifterSolenoid.set(Value.kForward);
		} else {
			RobotIO.shifterSolenoid.set(Value.kReverse);
		}

		if (state.grabberState) {
			RobotIO.grabberSolenoid.set(Value.kForward);
		} else {
			RobotIO.grabberSolenoid.set(Value.kReverse);
		}

		//----- DriveTrain -----
		double leftDistOffset = state.leftDist - RobotIO.leftEncoder.getDistance();
		double rightDistOffset = state.rightDist - RobotIO.rightEncoder.getDistance();
		double leftVelOffset = state.leftVel - RobotIO.leftEncoder.getRate();
		double rightVelOffset = state.rightVel - RobotIO.rightEncoder.getRate();
		double headingOffset = state.robotAngle - DriveSub.getHeading();
		double leftOutput = leftDistOffset * kPDrive + leftVelOffset * kDDrive + headingOffset * kPAngle;
		double rightOutput = rightDistOffset * kPDrive + rightVelOffset * kDDrive - headingOffset * kPAngle;
		RobotIO.leftDriveMotors.set(state.leftPwr + leftOutput);
		RobotIO.rightDriveMotors.set(state.rightPwr - rightOutput);

		//----- Elevator -----
		double elevDistOffset = state.elevPos - RobotIO.elevatorEncoder.getDistance();
		double elevVelOffset = state.elevVel - RobotIO.elevatorEncoder.getRate();
		double elevOutput = state.elevPwr + elevDistOffset * kPElev + elevVelOffset * kDElev;
		if (state.elevatorUp) {
			elevOutput = 0.3;
		} else if (state.elevatorDown) {
			elevOutput = -0.3;
		}
		ElevatorSub.setRate(elevOutput);

		//----- Grabber Angle -----
		double grabberOutput = state.grabberPower;
		if (state.grabberUp && !GrabberAngleSub.isUp()) {
			grabberOutput = Math.max(grabberOutput, 0.5);
		} else if (state.grabberDown && !GrabberAngleSub.isDown()) {
			grabberOutput = Math.min(grabberOutput, -0.5);
		}
		GrabberAngleSub.setRotateSpeed(grabberOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return stateArray.size() <= arrayPosition;
	}

	// Called once after isFinished returns true
	protected void end() {
		RobotIO.setVoltageComp(false);
		DriveSub.doNothing();

	}
}
