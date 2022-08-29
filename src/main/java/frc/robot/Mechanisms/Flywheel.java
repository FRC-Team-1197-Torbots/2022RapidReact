package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE FLYWHEEL SPEEDS
----------------------*/

import frc.robot.PID_Tools.*;

import java.util.ArrayList;

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

    /*
    private final double kP1 = 0.000006; //0.000006
    private final double kI1 = 0.000000; //0.000000
    private final double kD1 = 0.00000004; //0.00000004

    private final double kP2 = 0.0000008; //0.0000008
    private final double kI2 = 0.000000; //0.000000
    private final double kD2 = 0.00000001; //0.00000001
    */
    
    
    private final double kP1 = 0.0001; //0.00012 //0.0002
    private final double kI1 = 0.000012;//0.00001; //0.00001
    private final double kD1 = 0.000000;//0.000003; //0.0000002

    private final double kP2 = 0.0001; //0.00015
    private final double kI2 = 0.00001; //0.00001
    private final double kD2 = 0.000000;//0.000001;//0.0000015

    


    /********************
     New Distances:
     
     232.67 inches - 2770 rpm
     166.72 inches - 2450 rpm
     132.52 inches - 2200 rpm
     98.60 inches - 2100 rpm
     64.12 inches - X


     ********************/

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

    private ArrayList<Boolean> targetList = new ArrayList<Boolean>();

    public static enum runFlywheel {
        RUN, IDLE, OFF;
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
        pidIntegral = 0;
    }
    //-2250 - 154in
    //-1950 - 90in //min distance
    //-2450 - 198in //safe zone
    //-2800 - 220in

    public void run(runFlywheel flyState, double distance) {
        //System.out.printf("Current: %.2f Target: %.2f Ball? %s P: %.3f I: %.3f D: %.3f%n", currentSpeed, targetSpeed, Elevator.shooterBeam.get(), currentError * kP, pidIntegral * kI, pidDerivativeResult * kD);


        switch(flyState) {

            case RUN:
                /***
                 * FORMULAS TRIED:
                 * (-4.63f*distance) + -1534f
                 * (-5.18382f * distance) + -1562.89f --> first formula w/ new limelight angle
                 * (-6 * distance) - 1450f;
                 * (-5.3f + -1562.89f)
                 * 
                 */
                //System.out.println("Current Speed: " + currentSpeed);
                //System.out.println("Target Speed: " + targetSpeed);
                targetSpeed = -2000f;//(-5.7f * distance) + -1562.89f;    //(-4.63f*distance) + -1534f; //FORMULA FOR THE DISTANCE, MIGHT NEED TO CHANGE
                currentSpeed = flyEncoder.getVelocity();//rpm
                
                speedToSetMotor = pidRun(currentSpeed, targetSpeed);
                flyMotor.set(speedToSetMotor);
                flyMotor2.set(-speedToSetMotor);
                //System.out.println("Current speed: " + -currentSpeed);
                break;

            case IDLE:
                // targetSpeed = 0;
                // speedToSetMotor = pidRun(currentSpeed, targetSpeed);
                targetSpeed = -1500;
                currentSpeed = flyEncoder.getVelocity();
                speedToSetMotor = pidRun(currentSpeed, targetSpeed);
                flyMotor.set(speedToSetMotor);
                flyMotor2.set(-speedToSetMotor);
                //flyMotor.set(speedToSetMotor);
                OnTarget = false;

                //pidIntegral = 0;
                //sumSpeed = 0;
                break;
            case OFF:
                flyMotor.set(0);
                flyMotor2.set(0);
            break;
        }


        //System.out.println("Target speed: " + targetSpeed);
        //SmartDashboard.putNumber("Target Speed", -targetSpeed);
        //SmartDashboard.putNumber("Current Speed", -currentSpeed);
        //System.out.println("Current Speed: " + -currentSpeed);
        //SmartDashboard.putNumber("Error: ", -currentError);
        //SmartDashboard.putBoolean("OnTarget", OnTarget);
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
        //System.out.println("P: " + kP);

        if(Math.abs(currentError) < 50) {//80
            OnTarget = true;
        } else {
            OnTarget = false;

        }
        
        //System.out.println("PID Integral: " + pidIntegral);
        /*
        if(Math.abs(currentError) < 20) {
            pidIntegral = 0;
        }
        */

        /*
        if(pidIntegral * kI > 0.5) {
            pidIntegral = 0.5 / kI;
        } else if(pidIntegral * kI < -0.5) {
            pidIntegral = -0.5 / kI;
        }
        */
        
        // sumSpeed += ((currentError * kP) +
        // (pidIntegral * kI) +
        // (pidDerivativeResult * kD)); //+ FeedForward;
        incrementList(targetList);
        if(targetList.size() > 14 && targetSpeed < -1600){
            //System.out.println(targetReached());
        }
        

        return ((currentError * kP) +
         (pidIntegral * kI) +
         (pidDerivativeResult * kD));
    }

    public void stop() {
        flyMotor.set(0);
    }

    //TESTING TO TUNE PID, INPUTS HARD TARGET
    public void testRun(double rpm) {
        targetHighSpeed = rpm; //FORMULA FOR THE DISTANCE, MIGHT NEED TO CHANGE
        System.out.printf("Current: %.2f Target: %.2f Ball? %s P: %.3f I: %.3f D: %.3f%n", currentSpeed, targetHighSpeed, Elevator.shooterBeam.get(), currentError * kP, pidIntegral * kI, pidDerivativeResult * kD);

        // currentPosition = (adjustingConstant * flyEncoder1.getPosition()) / (gearRatio);
        // currentPosition = (adjustingConstant * 1) / (gearRatio);
        currentSpeed = flyEncoder.getVelocity();//rpm
        speedToSetMotor = pidRun(currentSpeed, targetHighSpeed);
        flyMotor.set(speedToSetMotor);
        flyMotor2.set(-speedToSetMotor);

        if(Math.abs(currentError) < 50) {//80
            OnTarget = true;
        } else {
            OnTarget = false;
        }

        //SmartDashboard.putNumber("Current speed", flyEncoder.getVelocity());
        //SmartDashboard.putNumber("Sum speed", sumSpeed);
        SmartDashboard.putNumber("Target RPM", targetHighSpeed);
        //System.out.println("RPM: " + flyEncoder.getVelocity());
    }

    public void setPIDValues(int state) {
        //STATE 1 IS FOR THE RAMPUP, STATE 2 IS FOR THE BALL 1 OFFSET
        kP = kP1;
        kI = kI1;
        kD = kD1;
        
        /*if (state == 1) {
            
        }
        else if (state == 2) {
            kP = kP2;
            kI = kI2;
            kD = kD2;
        }*/
    }

    private void incrementList(ArrayList<Boolean> list){
        list.add(OnTarget);
        if(list.size() > 15){
            list.remove(0);
        }
    }
    public boolean targetReached(){
        for(Boolean b: targetList){
            if(!b){
                return false;
            }
        }
        return true;
    }

    public void onDisable(){
        pidIntegral = 0;
        sumSpeed = 0;
        flyEncoder.setPosition(0);
    }
    
}
