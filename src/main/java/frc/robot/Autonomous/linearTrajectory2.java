package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PID_Tools.*;
import frc.robot.Drive.*;

public class linearTrajectory2 {
	private TorDrive drive;
	private double targetDistance;
	private double currentDistance;
	private boolean isFinished = false;
	
	private final double maxSpeed = 0.61;//0.6
	private final double minMaxSpeed = 0.1;
	private final double rampTime = 0.75;

	private double movingMaxSpeed = minMaxSpeed;
    
    //PID for translation
	private final double tkP = 0.4;//0.4
	private final double tkD = 0.0075;//0
    private final double tkI = 0.00;//0.00
    
    //PID For rotation
	
	private final double rkP = 0.3;//0.3
	private final double rkD = 0.0;//0.0
	private final double rkI = 0.02;//0.000
	
	private final double kF = 0.005;
	private final int lor = -1;
	private final int errorFob = 1;//forwards or backwards
	
	//tolerances
	private double positionTolerance = 0.02;//units: feet 0.04
	private double absolutePositionTolerance = 0.15;//units: 0.15 ft, or 1.8 inches
	private final double velocityTolerance = 0.001;//units: feet per second 0.05
	private final double headingTolerance = 2.5 * (Math.PI / 180.0);//units: radians 2.5 degrees
	
	//FOR THE CHEESE RUN
	//FORMULA: Acos(x) + b
	private final double minVelocity = 0.05;
	private final double maxVelocity = 0.35;
	private double kA = (maxVelocity - minVelocity) / 2;
	private double kB = minVelocity + kA;
	
	private double currentVelocity;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI = 0;
	
	private double vP;//velocity proportional
	private double vD;//velocity derivative
	private double vI = 0;
	
	private double omega;
	private double velocity;
	
	private double firstAngle;
	private double currentAngle;
	private double angleError;
	private double error;
	private double startDistance;
	private double lastTime;
	private double currentTime;
	private TorDerivative derivative;
	private TorDerivative accelDerivative;
	private TorDerivative angleDerivative;
	private double currentAcceleration = 0;
	
	private double timeOutTime;
	
	private boolean usePID;

	public static enum run {
		IDLE, GO;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public linearTrajectory2(TorDrive drive, double distance, double timeOutTime) {
		this.drive = drive;
		this.targetDistance = distance;
		this.timeOutTime = timeOutTime;
		derivative = new TorDerivative(kF);
		accelDerivative = new TorDerivative(kF);
		angleDerivative = new TorDerivative(kF);

		positionTolerance = distance;
		usePID = true;
	}
	
	public boolean isDone() {
		return isFinished;
	}
	
	public void init() {
		isFinished = false;
		runIt = run.GO;
		startDistance = 0;
		currentDistance = 0;
		firstAngle = drive.getHeading();
		currentAngle = drive.getHeading();
		angleError = currentAngle - firstAngle;
		//is in radians so we have to make sure that it goes from -pi to pi and does not have 
		//an absolute value greater than pi in order to be an efficient control system
		if(angleError > Math.PI) {
			angleError -= (2 * Math.PI);
		} else {
			if(angleError < -Math.PI) {
				angleError += (2 * Math.PI);
			}
		}
		angleDerivative.resetValue(0);
		derivative.resetValue(0);
		accelDerivative.resetValue(0);
		lastTime = Timer.getFPGATimestamp();

		vI = 0;

		drive.resetEncoder();
	}
		
	
	public void run() {
		currentAngle = drive.getHeading();
		//we can't fix current angle right now so that it can't be 359 degrees since we need it in this raw value first for the angleError
		//since it is never used other than for finding angleError, there is no need to make sure that it reads -1 degrees rather than 359 degrees
		currentDistance = drive.getPosition();
		currentTime = Timer.getFPGATimestamp();
		//System.out.println("Current angle: " + currentAngle);

		switch(runIt) {
		case IDLE:
			break;
		case GO:

			//SmartDashboard.putNumber("Lineartraj Distance: ", currentDistance);
			angleError = currentAngle - firstAngle;
			//is in radians so we have to make sure that it goes from -pi to pi and does not have 
			//an absolute value greater than pi in order to be an efficient control system
			if(angleError > Math.PI) {
				angleError -= (2 * Math.PI);
			} else {
				if(angleError < -Math.PI) {
					angleError += (2 * Math.PI);
				}
			}

			
			omegaP = angleError * rkP;
			omegaI += angleError;
			if(Math.abs(angleError) < headingTolerance) {
				omegaI = 0;
			}
			if(omegaI > ((0.25) / (rkI * kF))) {
				omegaI = ((0.5) / (rkI * kF));
			}
			if(omegaI < -((0.25) / (rkI * kF))) {
				omegaI = -((0.5) / (rkI * kF));
			}
			
			omegaD = (angleDerivative.estimate(angleError)) * rkD;
			omega = omegaP + omegaD + (omegaI * rkI * kF);
			omega *= lor;
			
			
			//since this distance is always positive, we have to multiply by fob for if it is negative
			error = targetDistance - currentDistance;//error always positive if approaching
			
			vI += error;

			if(Math.abs(error) <= positionTolerance * 0.5) {
				vI = 0;
			}

			if(vI > (0.5 / (tkI * kF))) {
				vI = (0.5 / (tkI * kF));
			}
			if(vI < -(0.5 / (tkI * kF))) {
				vI = -(0.5 / (tkI * kF));
			}

			vP = error * tkP;//vP gets smaller bc error decreases

			currentVelocity = derivative.estimate(drive.getPosition());//almost always positive
			SmartDashboard.putNumber("current velocity", currentVelocity);

			//has to be multiplied by -1 so that if it is approaching the target to fast
			//it does not act as a positive. Because, if it was approaching fast, the
			//derivative would be positive
			vD = currentVelocity * tkD;//degrees per second
			velocity = vP + vD + (vI * tkI * kF);

			if((currentTime-lastTime) < rampTime) {
				movingMaxSpeed = minMaxSpeed + (maxSpeed - minMaxSpeed) * (currentTime - lastTime) / rampTime;
			}

			if(velocity > movingMaxSpeed) {
				velocity = movingMaxSpeed;
			} else if(velocity < -movingMaxSpeed) {
				velocity = -movingMaxSpeed;
			}

			
			
			drive.setMotorSpeeds(velocity+omega, velocity-omega);// + omega, velocity - omega);//right, left	
			
			//0.099x -0.163
			if((Math.abs(error) <= (positionTolerance + absolutePositionTolerance) 
				&& Math.abs(velocity) < velocityTolerance 
				&& Math.abs(angleError) <= headingTolerance)  
				// currentDistance >= targetDistance - ((0.099 * targetDistance))
				|| (currentTime - lastTime > timeOutTime))
			{
				drive.setMotorSpeeds(0, 0);
				isFinished = true;
				runIt = run.IDLE;
			}
			break;
			
		}
			
	}

	//Runs without PID, uses a graph to plot out the velocities
	//let's cheese it
	// public void cheeseRun() {
	// 	//double FeedForward;
	// 	switch(runIt) {
	// 		case IDLE:
	// 			break;
	// 		case GO:
	// 			//System.out.println("Velocity: " + velocity);
	// 			//System.out.println("Current distance: " + drive.getAverageEncoderPosition());

	// 			error = targetDistance - currentDistance;

	// 			SmartDashboard.putNumber("heading", drive.getHeading());

	// 			currentDistance = drive.getPosition();
	// 			velocity = -kA * Math.cos((Math.PI * currentDistance) / (targetDistance / 2)) + kB;
	// 			if(velocity > maxSpeed) {
	// 				velocity = maxSpeed;
	// 			} else if(velocity < -maxSpeed) {
	// 				velocity = -maxSpeed;
	// 			}
	// 			drive.setMotorSpeeds(velocity, velocity);

	// 			if((Math.abs(error) <= positionTolerance
	// 				&& Math.abs(angleError) <= headingTolerance
	// 				&& Math.abs(currentVelocity) < velocityTolerance)
	// 				|| (currentTime - lastTime > timeOutTime))
	// 			{
	// 				drive.setMotorSpeeds(0, 0);
	// 				isFinished = true;
	// 				System.out.println("Finished");
	// 				runIt = run.IDLE;
	// 			}
		// }
		
	// }
}

/* OLD PID

vI += error;
			
			if(Math.abs(error) <= positionTolerance * 0.5) {
				vI = 0;
			}

			if(vI > (0.7 / (tkI * kF))) {
				vI = (0.7 / (tkI * kF));
			}
			if(vI < -(0.7 / (tkI * kF))) {
				vI = -(0.7 / (tkI * kF));
			}

			vP = error * tkP;//vP gets smaller bc error decreases
			
			
			currentVelocity = derivative.estimate(drive.getPosition());//almost always positive
			currentAcceleration = -accelDerivative.estimate(currentVelocity);
			
			//has to be multiplied by -1 so that if it is approaching the target to fast
			//it does not act as a positive. Because, if it was approaching fast, the
			//derivative would be positive
			vD = currentVelocity * tkD;//degrees per second
			velocity = vP + vD + (vI * tkI);
*/