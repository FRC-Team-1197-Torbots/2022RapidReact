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
    private final double targetHighSpeed = -2000f;// rpm 5676 theoretical max
    private final double targetLowSpeed = 1000f;//rpm
    private final double highSpeedConstant = 0.0;//0.9
    private final double lowSpeedConstant = 0.0;
    // private final double adjustingConstant = 1.0 / 3;
    private final double kP = 0.1;//.00035
    private final double kI = 0.0;//.000005
    private final double kD = 0.0; //original 0.5 0 0
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
    private CANSparkMax flywheelMotor1;
    private CANSparkMax otherFlywheelMotor;
    private CANEncoder flywheelEncoder1;
    // private CANEncoder flywheelEncoder2;
    private XboxController player2;

    // time stuff to make sure only goes in correct intervals
    private long currentTime;
    private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    private long lastCountedTime;
    private boolean starting = true;

    public Flywheel(XboxController player2) {
        // this.flywheelMotor1 = flywheelMotor1;
        // this.otherFlywheelMotor = flywheelMotor2;
        // this.flywheelMotor2.follow(this.flywheelMotor1);
        // this.flywheelEncoder1 = this.flywheelMotor1.getEncoder();
        // this.flywheelEncoder2 = this.flywheelMotor2.getEncoder();
        flywheelMotor1 = new CANSparkMax(8, MotorType.kBrushless);
        otherFlywheelMotor = new CANSparkMax(7, MotorType.kBrushless);
        flywheelEncoder1 = otherFlywheelMotor.getEncoder();

        this.player2 = player2;
        // findCurrentSpeed = new TorDerivative(dt);
        // findCurrentSpeed.resetValue(0);
        pidDerivative = new TorDerivative(dt);
        pidDerivative.resetValue(0);
        
    }



    public void init(){

    }



    /*
    COMMENTED OUT FOR NOW
    public void revUp(double distance){
        //Calculate targetSpeed through distance ranges
        //use PID to constantly increase the speed to a target value (probably reuse Brennan's code)

        motor1.set(0.5);
        motor2.set(0.5);

    }

    public void stop(){
        //set motor speed to 0
        motor1.set(0);
        motor2.set(0);
    }
    */

    public void run(boolean run, boolean forceOn) {
        //testing high speed
        if (run) {
            targetSpeed = targetHighSpeed;
            FeedForward = targetSpeed/MaxMotorSpeed;
            
            // currentPosition = (adjustingConstant * flywheelEncoder1.getPosition()) / (gearRatio);
            // currentPosition = (adjustingConstant * 1) / (gearRatio);
            currentSpeed = flywheelEncoder1.getVelocity() / MaxMotorSpeed;//rpm
            speedToSetMotor = pidRun(currentSpeed, FeedForward);

            System.out.println("RPM: " + currentSpeed);
            System.out.println("Feedforward " + FeedForward);
           
            flywheelMotor1.set(speedToSetMotor);
            otherFlywheelMotor.set(speedToSetMotor);
        }
        else {
            flywheelMotor1.set(0 * 1.0f);
            //System.out.println("RPM: " + flywheelEncoder1.getVelocity());
            otherFlywheelMotor.set(-0 * 1.0f);
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
                // currentPosition = (adjustingConstant * flywheelEncoder1.getPosition()) / (gearRatio);
                // currentPosition = (adjustingConstant * 1) / (gearRatio);
                currentSpeed = -flywheelEncoder1.getVelocity() / gearRatio;//rpm
                speedToSetMotor = pidRun(currentSpeed, targetSpeed) + highSpeedConstant;
            } else {   
                targetSpeed = targetLowSpeed;
                // currentPosition = (adjustingConstant * flywheelEncoder1.getPosition()) / (gearRatio);
                // currentPosition = (adjustingConstant * 1) / (gearRatio);
                currentSpeed = -flywheelEncoder1.getVelocity() / gearRatio;//rpm
                speedToSetMotor = pidRun(currentSpeed, targetSpeed) + lowSpeedConstant;
                speedToSetMotor = lowSpeedConstant;
            }
            if(run) {

                if(player2.getRawButton(6) || forceOn) {
                    flywheelMotor1.set(speedToSetMotor * 1.0f);
                    otherFlywheelMotor.set(-speedToSetMotor * 1.0f);
                    // flywheelMotor1.set(0.8f);
                    // otherFlywheelMotor.set(-0.8f);
                    // flywheelMotor1.set(0.0f);
                    // flywheelMotor2.set(0.0f);
                } else {
                    // flywheelMotor1.set(-0.5f);
                    // flywheelMotor2.set(0.5f);
                    flywheelMotor1.set(lowSpeedConstant * 1.0f);
                    otherFlywheelMotor.set(lowSpeedConstant * -1.0f);
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
    
}
