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
    private final double targetLowSpeed = 200f;//rpm
    private final double highSpeedConstant = 0.0;//0.9
    private final double lowSpeedConstant = 0.0;
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
    private CANEncoder flyEncoder1;
    // private CANEncoder flyEncoder2;

    // time stuff to make sure only goes in correct intervals
    private long currentTime;
    private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    private long lastCountedTime;
    private boolean starting = true;

    public Flywheel() {
       
        flyMotor = new CANSparkMax(7, MotorType.kBrushless);
        this.flyEncoder1 = flyMotor.getEncoder();
        pidDerivative = new TorDerivative(dt);
        pidDerivative.resetValue(0);
        
    }

    public void init(){

    }

    public void run(boolean run, boolean forceOn, double distance) {
        //testing high speed
        if (run) {
            targetHighSpeed = -((108 * distance) + 1776); //FORMULA FOR THE DISTANCE
            targetSpeed = targetHighSpeed;
            FeedForward = targetSpeed/MaxMotorSpeed;
            
            // currentPosition = (adjustingConstant * flyEncoder1.getPosition()) / (gearRatio);
            // currentPosition = (adjustingConstant * 1) / (gearRatio);
            currentSpeed = flyEncoder1.getVelocity() / MaxMotorSpeed;//rpm
            speedToSetMotor = pidRun(currentSpeed, FeedForward);

            System.out.println("RPM: " + (int)(currentSpeed * 5676));
            //System.out.println("Feedforward " + FeedForward);
           
            flyMotor.set(speedToSetMotor);
        }
        else {
            flyMotor.set(0 * 1.0f);
            sumSpeed = 0;
            //System.out.println("RPM: " + flyEncoder1.getVelocity());
            //upperMotor.set(-0 * 1.0f);
        }

        
        //OLD CODE
        /*currentTime = (long) (Timer.getFPGATimestamp() * 1000);
        if (((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > // has current time minus start time to see the relative time the trajectory has been going
            ((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) // subtracts that mod dt times 1000 so that it is floored to
            // the nearest multiple of dt times 1000 then checks if that is greater than the last one to see if it is time to move on to the next tick
            || starting) {
            starting = false;
            lastCountedTime = currentTime;
            if(player2.getRawButton(6) || forceOn) {
                targetSpeed = targetHighSpeed;
                // currentPosition = (adjustingConstant * flyEncoder1.getPosition()) / (gearRatio);
                // currentPosition = (adjustingConstant * 1) / (gearRatio);
                currentSpeed = -flyEncoder1.getVelocity() / gearRatio;//rpm
                speedToSetMotor = pidRun(currentSpeed, targetSpeed) + highSpeedConstant;
            } else {   
                targetSpeed = targetLowSpeed;
                // currentPosition = (adjustingConstant * flyEncoder1.getPosition()) / (gearRatio);
                // currentPosition = (adjustingConstant * 1) / (gearRatio);
                currentSpeed = -flyEncoder1.getVelocity() / gearRatio;//rpm
                speedToSetMotor = pidRun(currentSpeed, targetSpeed) + lowSpeedConstant;
                speedToSetMotor = lowSpeedConstant;
            }
            if(run) {

                if(player2.getRawButton(6) || forceOn) {
                    flyMotor.set(speedToSetMotor * 1.0f);
                    upperMotor.set(-speedToSetMotor * 1.0f);
                    // flyMotor.set(0.8f);
                    // upperMotor.set(-0.8f);
                    // flyMotor.set(0.0f);
                    // flywheelMotor2.set(0.0f);
                } else {
                    // flyMotor.set(-0.5f);
                    // flywheelMotor2.set(0.5f);
                    flyMotor.set(lowSpeedConstant * 1.0f);
                    upperMotor.set(lowSpeedConstant * -1.0f);
                }
            }
            
            // SmartDashboard.putNumber("current Speed", currentSpeed);
        }*/
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
