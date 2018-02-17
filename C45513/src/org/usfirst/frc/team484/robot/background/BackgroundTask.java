package org.usfirst.frc.team484.robot.background;

/**
 * {@code BackgroundTask} wraps a Thread, allowing easy control of its execution.
 */
public abstract class BackgroundTask {
	private boolean firstRun = true;
	private volatile boolean running = false;
	
	protected Thread thread;
	protected long intervalTimeMs = 1;
	
	/**
	 * Constructs a BackgroundTask with the default interval of 1 ms
	 */
	public BackgroundTask() {
		thread = new Thread() {
			@Override
			public void run() {
				while (!isInterrupted()) {
					setup();
					
					while (running) {
						// Run execute and measure the time it took to do so, breaking out of the loop
						// if an error occurred
						long startTime = System.currentTimeMillis();
						
						execute();
						
						long dt = System.currentTimeMillis() - startTime;
						
						// Sleep for the time remaining in the current time window, so we keep the
						// interval relatively regular
						try {
							Thread.sleep(Math.max(intervalTimeMs - dt, 0));
						} catch (InterruptedException e) {
							// Attempt to clean up if we get interrupted
							break;
						}
					}
					
					cleanup();
					
					// Wait until enabled, sleeping for 10ms each time
					while (!running) {
						try {
							Thread.sleep(10);
						} catch (InterruptedException e) { break; }	
					}
				}
				
				lastCleanup();
			}
		};
	}
	
	/**
	 * Constructs a BackgroundTask with an explicit interval time.
	 * @param interval The time between invocations of {@link #execute}
	 */
	public BackgroundTask(long interval) {
		this();
		intervalTimeMs = interval;
	}
	
	/**
	 * Enables the internal thread. If it is not so, the thread will be started and the running flag
	 * will be set to true.
	 */
	public final void enable() {
		if (firstRun) thread.start();
		running = true;
		firstRun = false;
	}
	
	/**
	 * Disables the internal thread. The running flag will be set to false.
	 */
	public final void disable() {
		running = false;
	}
	
	/**
	 * 
	 */
	public final void kill() {
		// Notify the thread that we are shutting down
		thread.interrupt();
		disable();
	}
	
	/**
	 * Sets the minimum time between the invocations of execute.
	 * @param interval
	 */
	public void setInterval(long interval) {
		intervalTimeMs = interval;
	}
	
	public long getInterval() {
		return intervalTimeMs;
	}
	
	/**
	 * Called every "tick" of the internal thread.
	 * @throws Throwable Any exception thrown here will stop execution of the current execution loop
	 * 		   and attempt to call cleanup and start back at setup.
	 */
	protected abstract void execute();
	
	/**
	 * Called once before the main execution loop.
	 * @throws Throwable Any exception thrown here will abort the internal thread.
	 */
	protected void setup() {}
	
	/**
	 * Called once after the main execution loop.
	 * @throws Throwable Any exception thrown here will abort the internal thread.
	 */
	protected void cleanup() {}
	
	/**
	 * Called once after the task is killed, for final cleanup/destruction. A similar firstSetup
	 * does not exist because of potential observation of a half-constructed object.
	 * @throws Throwable
	 */
	protected void lastCleanup() {}
}
