package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE FLYWHEEL SPEEDS
----------------------*/

import frc.robot.PID_Tools.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel {
    //2950 rpm is shot from 11ft. kp = 0.02 i = 0.4, d = 0
    //2400 rpm is shot from 6 ft. kp = 0.02, i = 0.4, d = 0
    //3500 rpm is shot from 16 ft. kp = 0.02, kI = 0.4, kD = 0
    private double targetHighSpeed;// rpm 5676 theoretical max
    //private final double targetLowSpeed = 200f;//rpm
    //private final double highSpeedConstant = 0.0;//0.9
    //private final double lowSpeedConstant = 0.0;
    // RPM below 2000 p = 0.045 i = 0.4 d = 0
    private final double kP1 = 0.000006;//.00035//0.05
    private final double kI1 = 0.000000;//0.00000006;//.000005
    private final double kD1 = 0.00000004; //original 0.5 0 0

    private final double kP2 = 0.0000008;//.00035//0.05
    private final double kI2 = 0.000000;//0.00000006;//.000005
    private final double kD2 = 0.00000001; //original 0.5 0 0

    private double kP;
    private double kI;
    private double kD;

    private double FeedForward;
    private final double MaxMotorSpeed = 4500;
    private double currentError = 0;

    private TorDerivative pidDerivative;
    private double pidDerivativeResult;

    public double pidIntegral = 0;

    private double targetSpeed;
    private double currentSpeed;
    private double speedToSetMotor;
    private double sumSpeed = 0;
    // private TorDerivative findCurrentSpeed;
    // private double currentPosition;
    private final double gearRatio = 1;// ratio from encoder to flywheel
    private CANSparkMax flyMotor;
    private CANSparkMax flyMotor2;
    //private CANSparkMax upperMotor;
    private RelativeEncoder flyEncoder;
    // private CANEncoder flyEncoder2;

    // time stuff to make sure only goes in correct intervals
    //private long currentTime;
    //private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    public boolean OnTarget = false;

    public static enum runFlywheel {
        RUN, IDLE;
    }

    public Flywheel() {

        flyMotor = new CANSparkMax(12, MotorType.kBrushless);
        flyMotor2 = new CANSparkMax(16, MotorType.kBrushless);
        flyEncoder = flyMotor.getEncoder();
       
        pidDerivative = new TorDerivative(dt);
        pidDerivative.resetValue(0);
        
    }

    public void init(){
        setPIDValues(1);
    }
    //-2250 - 154in
    //-1950 - 90in //min distance
    //-2450 - 198in //safe zone
    //-2800 - 220in

    public void run(runFlywheel flyState, double distance) {
        switch(flyState) {
            case RUN:
                targetSpeed = (-4.63f*distance) + -1534f; //FORMULA FOR THE DISTANCE, MIGHT NEED TO CHANGE
                currentSpeed = flyEncoder.getVelocity();//rpm
                speedToSetMotor = pidRun(currentSpeed, targetSpeed);
                flyMotor.set(speedToSetMotor);
                flyMotor2.set(-speedToSetMotor);
                //System.out.println("Current speed: " + -currentSpeed);
                break;

            case IDLE:
                // targetSpeed = 0;
                // speedToSetMotor = pidRun(currentSpeed, targetSpeed);
                flyMotor.set(0);
                flyMotor2.set(0);
                OnTarget = false;

                pidIntegral = 0;
                sumSpeed = 0;
                break;
        }


        //System.out.println("Target speed: " + targetSpeed);
        SmartDashboard.putNumber("Target Speed", -targetSpeed);
        SmartDashboard.putNumber("Current Speed", -currentSpeed);
        //System.out.println("Current Speed: " + -currentSpeed);
        SmartDashboard.putNumber("Error: ", -currentError);
        SmartDashboard.putBoolean("OnTarget", OnTarget);
        //System.out.println("Percentage: " + (currentSpeed/targetSpeed));
        //System.out.println("Time: " + Timer.getFPGATimestamp());
    }

    
    public boolean isFastEnough() {
        return currentSpeed > 0.95 * targetHighSpeed;
    }

    public double pidRun(double currentSpeed, double targetSpeed) {

        currentError = targetSpeed - currentSpeed;
        
        // SmartDashboard.putNumber("currentError:", currentError);
        pidDerivativeResult = pidDerivative.estimate(currentError);
        pidIntegral += currentError;
        System.out.println("P: " + kP);

        if(Math.abs(currentError) < 80) {
            OnTarget = true;
        } else {
            OnTarget = false;
        }
            

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

    //TESTING TO TUNE PID, INPUTS HARD TARGET
    public void testRun(double rpm) {
        targetHighSpeed = rpm; //FORMULA FOR THE DISTANCE, MIGHT NEED TO CHANGE
        targetSpeed = targetHighSpeed;
        FeedForward = targetSpeed/MaxMotorSpeed;
        
        // currentPosition = (adjustingConstant * flyEncoder1.getPosition()) / (gearRatio);
        // currentPosition = (adjustingConstant * 1) / (gearRatio);
        currentSpeed = flyEncoder.getVelocity() / MaxMotorSpeed;//rpm
        speedToSetMotor = pidRun(currentSpeed, FeedForward);
        flyMotor.set(speedToSetMotor);

        SmartDashboard.putNumber("Current speed", flyEncoder.getVelocity());
        SmartDashboard.putNumber("Sum speed", sumSpeed);
        //System.out.println(flyEncoder.getVelocity());
    }

    public void setPIDValues(int state) {
        //STATE 1 IS FOR THE RAMPUP, STATE 2 IS FOR THE BALL 1 OFFSET
        if (state == 1) {
            kP = kP1;
            kI = kI1;
            kD = kD1;
        }
        else if (state == 2) {
            kP = kP2;
            kI = kI2;
            kD = kD2;
        }
    }

    public void onDisable(){
        pidIntegral = 0;
        sumSpeed = 0;
        flyEncoder.setPosition(0);
    }
    
}
