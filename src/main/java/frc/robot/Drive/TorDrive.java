package frc.robot.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class TorDrive {//this is the upper layer of the drive
	private DriveController ActiveController;
	// private MotionProfileDriveController MPDC;//Motion Profiling Drive Controller
	private DriveController ADC;//Arcade Drive Controller
	private DriveController TDC;//Test Drive Controller
	private DriveHardware hardware;
	private XboxController player1;
	private double leftOutput;
	private double rightOutput;
	
	private static enum driveStateRunner {
		// MotionProfile,
		ArcadeDrive,
		TestDrive;
		private driveStateRunner() {}
	}
	private driveStateRunner drive = driveStateRunner.TestDrive;
	
	public TorDrive(DriveHardware hardware, XboxController player1) {
		this.player1 = player1;
		this.hardware = hardware;
		ActiveController = new TestDriveController(hardware, player1);//starts out as nothing
		// MPDC = new MotionProfileDriveController(hardware, player1);
		ADC = new ArcadeDriveController(hardware, player1);
		//TDC = new TestDriveController(hardware, player1);
	}

	public void Run() {
		ADC.run();
	}
	
	public void Run(boolean testing, boolean isTeleop) { //boolean topLimeLight
		//ADC.limeLightTop(topLimeLight);
		//TDC.limeLightTop(topLimeLight);
		//if(testing) {
			drive = driveStateRunner.TestDrive;
		// } else {
		
		//runs all of them to update their values
		// MPDC.run();
		ADC.run();
		// TDC.run();
		
		//sets the activeController to one of them so the active controller will get all the values
		switch(drive) {
		// case MotionProfile:
		// 	ActiveController = MPDC;
		// 	break;
			case ArcadeDrive:
				ActiveController = ADC;
				break;
			case TestDrive:
				ActiveController = TDC;
				break;
		}
		//gets the values from the active controller and sets them to the hardware
		leftOutput = ActiveController.getLeftOutput();
		rightOutput = player1.getRawAxis(1);
		hardware.setMotorSpeeds(rightOutput, leftOutput);
	}

	public void setMotorSpeeds(double speed1, double speed2) {//for auto
		hardware.setMotorSpeeds(speed1, speed2);
	}

	public double getHeading() {
		return hardware.getHeading();
	}

	public double getAverageEncoderPosition() {
		return hardware.getAverageEncoderPosition();
	}

	public double getPosition() {
		return hardware.getPosition();
	}

	public double getLeftEncoder() {
		return hardware.getLeftEncoder();
	}

	public double getRightEncoder() {
		return hardware.getRightEncoder();
	}

	public void resetEncoder() {
		hardware.resetEncoder();
	}

	public void init() {
		ADC.init();
		TDC.init();
		hardware.init();
	}

	public void resetGyro() {
		hardware.resetGyro();
	}
	
	//we are only running trajectories in MP so that is why it only takes from the MPDC
	// public boolean MPTrajectoryIsComplete() {
	// 	return MPDC.activeTrajectoryIsComplete();
	// }

	//we are only running trajectories in MP so that is why it only takes from the MPDC
	// public void executeTrajectory(TorTrajectory trajectory, long millisecondsToTimeOut) {
	// 	MPDC.executeTrajectory(trajectory, millisecondsToTimeOut);
	// }
}