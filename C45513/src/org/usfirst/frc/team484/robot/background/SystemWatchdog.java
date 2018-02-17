package org.usfirst.frc.team484.robot.background;

import java.util.ArrayList;
import java.util.HashSet;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.TogglableSpeedController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SystemWatchdog extends BackgroundTask {
	private MotorTracker[] driveWatchdogs = new MotorTracker[RobotSettings.LEFT_DRIVE_MOTOR_IDS.length + RobotSettings.RIGHT_DRIVE_MOTOR_IDS.length];
	private HashSet<MotorTracker> aliveSet = new HashSet<>(), deadSet = new HashSet<>();
	
	private static Side getMotorSide(int id) {
		for (int leftId : RobotSettings.LEFT_DRIVE_MOTOR_IDS) if (id == leftId) return Side.LEFT;
		for (int rightId : RobotSettings.RIGHT_DRIVE_MOTOR_IDS) if (id == rightId) return Side.RIGHT;
		return null;
	}
	
	// A motor descriptor/tracker. Contains tracking information like a buffer of past
	// currents from the PDP along with information about the motor it is currently tracking
	private static class MotorTracker {
		private static final int BUFFER_SIZE = 20;
		private static final double MOTOR_ACTIVE_THRESHOLD = 1.0;
		private static final double CURRENT_THRESHOLD = 1.0;
		
		private double[] currentBuffer = new double[BUFFER_SIZE];
		private TogglableSpeedController<WPI_TalonSRX> motorRef;
		private Side motorSide;
		public boolean deadFlag = false;
		public final int pdpModule;
		
		public MotorTracker(TogglableSpeedController<WPI_TalonSRX> motorRef, int pdpModule) {
			this.motorRef = motorRef;
			this.pdpModule = pdpModule;
			motorSide = SystemWatchdog.getMotorSide(pdpModule);
		}
		
		public void updateCurrentBuffer() {
			// Shift the array over one, leaving the first slot available by copying the
			// array from 0..n-1 to 1..n
			System.arraycopy(currentBuffer, 0, currentBuffer, 1, currentBuffer.length - 1);
			// Update first entry
			currentBuffer[0] = RobotIO.pdp.getCurrent(pdpModule);
		}
		
		public double bufferAverage() {
			int sum = 0;
			for (double current : currentBuffer) sum += current;
			return sum / (double) currentBuffer.length;
		}
		
		public boolean isDead() {
			return motorRef.get() >= MOTOR_ACTIVE_THRESHOLD && bufferAverage() <= CURRENT_THRESHOLD;
		}
	}
	
	private static enum Side {
		RIGHT, LEFT;
		
		Side opposite() {
			switch (this) {
			case RIGHT: return LEFT;
			case LEFT: return RIGHT;
			}
			// unreachable
			return null;
		}
	}
	
	public SystemWatchdog(long interval) {
		super(interval);
		ArrayList<MotorTracker> trackers = new ArrayList<>();
		
		for (int i = 0; i < RobotSettings.LEFT_DRIVE_MOTOR_IDS.length; i++) {
			MotorTracker tracker =
					new MotorTracker(RobotIO.leftTalons.get(i), RobotSettings.LEFT_DRIVE_MOTOR_IDS[i]);
			trackers.add(tracker);
			aliveSet.add(tracker);
		}
		for (int i = 0; i < RobotSettings.RIGHT_DRIVE_MOTOR_IDS.length; i++) {
			MotorTracker tracker =
					new MotorTracker(RobotIO.rightTalons.get(i), RobotSettings.RIGHT_DRIVE_MOTOR_IDS[i]);
			trackers.add(tracker);
			aliveSet.add(tracker);
		}

		driveWatchdogs = trackers.toArray(driveWatchdogs);
	}
	
	private void setMotorAlive(MotorTracker tracker, boolean alive) {
		if (alive) {
			tracker.motorRef.enable();
			if (deadSet.remove(tracker)) aliveSet.add(tracker);
		} else {
			tracker.motorRef.disable();
			if (aliveSet.remove(tracker)) deadSet.add(tracker);
		}
	}
	
	private MotorTracker getMotorOnSide(boolean alive, Side side) {
		HashSet<MotorTracker> set = alive ? aliveSet : deadSet;
		return set.stream().filter(item -> item.motorSide.equals(side)).findFirst().orElse(null);
	}
	
	private void notifyDrivers(MotorTracker tracker, boolean alive) {
		// TODO: unimplemented
		System.out.println(tracker + ": alive = " + alive);
	}
	
	@Override
	protected void execute() {
		// Loop through all the drivetrain watchdogs, update them, and disable any motors if need be
		for (MotorTracker tracker : driveWatchdogs) {
			tracker.updateCurrentBuffer();
			
			// Status went from alive -> dead
			if (tracker.isDead() && !tracker.deadFlag) {
				setMotorAlive(tracker, false);
				setMotorAlive(getMotorOnSide(true, tracker.motorSide.opposite()), false);
				notifyDrivers(tracker, false);
				
				// Prevent this block from being executed again
				tracker.deadFlag = true;
			}
			
			// Somehow motor came back to life! Zombie motor!
			if (!tracker.isDead() && tracker.deadFlag) {
				setMotorAlive(tracker, true);
				setMotorAlive(getMotorOnSide(false, tracker.motorSide.opposite()), true);
				notifyDrivers(tracker, true);
				
				// Prevent this block from being executed again
				tracker.deadFlag = false;
			}
		}
	}

}
