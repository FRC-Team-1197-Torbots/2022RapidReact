package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE FLYWHEEL SPEEDS
----------------------*/

import frc.robot.PID_Tools.*;

// import com.revrobotics.AlternateEncoderType;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.IMotorController;
// import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.EncoderType;
// import com.revrobotics.EncoderType;
// import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel {
    //2950 rpm is shot from 11ft. kp = 0.02 i = 0.4, d = 0
    //2400 rpm is shot from 6 ft. kp = 0.02, i = 0.4, d = 0
    //3500 rpm is shot from 16 ft. kp = 0.02, kI = 0.4, kD = 0
    private double targetHighSpeed;// rpm 5676 theoretical max
    //private final double targetLowSpeed = 200f;//rpm
    //private final double highSpeedConstant = 0.0;//0.9
    //private final double lowSpeedConstant = 0.0;
    // RPM below 2000 p = 0.045 i = 0.4 d = 0
    private final double kP = 0.02;//.00035//0.05
    private final double kI = 0.4;//.000005
    private final double kD = 0.00; //original 0.5 0 0
    private double FeedForward;
    private final double MaxMotorSpeed = 5676f;
    private double currentError = 0;

    private TorDerivative pidDerivative;
    private double pidDerivativeResult;

    private double pidIntegral = 0;

    private double targetSpeed;
    private double currentSpeed;
    private double speedToSetMotor;
    private double sumSpeed = 0;
    // private TorDerivative findCurrentSpeed;
    // private double currentPosition;
    private final double gearRatio = 1;// ratio from encoder to flywheel
    private CANSparkMax flyMotor;
    //private CANSparkMax upperMotor;
    private RelativeEncoder flyEncoder;
    // private CANEncoder flyEncoder2;

    // time stuff to make sure only goes in correct intervals
    //private long currentTime;
    //private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    //private long lastCountedTime;
    //private boolean starting = true;

    public static enum runFlywheel {
        RUN, IDLE;
    }

    public Flywheel() {

        //flyMotor = new CANSparkMax(1, MotorType.kBrushless);
        flyEncoder = flyMotor.getEncoder();
       
        pidDerivative = new TorDerivative(dt);
        pidDerivative.resetValue(0);
        
    }

    public void init(){

    }
    

    public void run(runFlywheel flyState, double distance) {
        switch(flyState) {
            case RUN:
                targetHighSpeed = -((108 * distance) + 1776); //FORMULA FOR THE DISTANCE, MIGHT NEED TO CHANGE
                targetSpeed = targetHighSpeed;
                FeedForward = targetSpeed/MaxMotorSpeed;
                
                // currentPosition = (adjustingConstant * flyEncoder1.getPosition()) / (gearRatio);
                // currentPosition = (adjustingConstant * 1) / (gearRatio);
                currentSpeed = flyEncoder.getVelocity() / MaxMotorSpeed;//rpm
                speedToSetMotor = pidRun(currentSpeed, FeedForward);
                flyMotor.set(speedToSetMotor);
            case IDLE:
                flyMotor.set(0 * 1.0f);
        }
    }

    
    public boolean isFastEnough() {
        return currentSpeed > 0.95 * targetHighSpeed;
    }

    public double pidRun(double currentSpeed, double targetSpeed) {
        
        currentError = targetSpeed - currentSpeed;
        
        // SmartDashboard.putNumber("currentError:", currentError);
        pidDerivativeResult = pidDerivative.estimate(currentError);
        pidIntegral += currentError;

        if(currentError < 20) {
            pidIntegral = 0;
        }

        if(pidIntegral * kI > 0.5) {
            pidIntegral = 0.5 / kI;
        } else if(pidIntegral * kI < -0.5) {
            pidIntegral = -0.5 / kI;
        }

        sumSpeed += ((currentError * kP) +
        (pidIntegral * kI) +
        (pidDerivativeResult * kD)); //+ FeedForward;

        return sumSpeed;
    }

    public void stop() {
        flyMotor.set(0);
    }
    
}
