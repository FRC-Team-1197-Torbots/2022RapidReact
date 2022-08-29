package frc.robot.Drive;

import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

public class ArcadeDriveController extends DriveController {

//    private DigitalInput colorWheelDIO;
   private double throttleAxis;
   private double arcadeSteerAxis;
   private double leftOutput;
   private double rightOutput;
   private double rightMotorSpeed;
   private double leftMotorSpeed;
   private XboxController player1;
   private DriveHardware hardware;

   private double distance;
   private HelicopTORPID limeLightPID;
   private double currentVelocity;
   private TorDerivative findCurrentVelocity;

   //time stuff to make sure only goes in correct intervals
   private long currentTime;
   private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
   private double timeInterval = 0.005;
   private double dt = timeInterval;
   private long lastCountedTime;
   private boolean starting = true;

   /*
    * tuneable
    * things---------------------------------------------------------------->>>>>>>
    * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */
   private final double areaAt1Meter = 1.27;//in percent//maybe later?

   // PID
   // Stuff------------------------------>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

   private final double velocitykP = 0.0000125;// velocity stuff probably not needed at all and should keep 0
   private final double velocitykI = 0.0;
   private final double velocitykD = 0.0000008;

   // ------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


   private double currentError = 0;
   private TorDerivative pidDerivative;
   private double pidDerivativeResult;
   private double pidIntegral = 0;
   private double LeftPIDSum = 0, RightPIDSum = 0;

   private double throttle = 0;
   private double steer = 0;
   
   private final double MAX_VELOCITY = 50000f;//45000f; //28000 //39000 // 34,000 // 39,000
   private final double STEER_SCALAR = 1.15; //1.2
   //this is for the curve drive

   private final double POSRANGE_MAX_ACCEL = 0.02;//0.03;//0.08;
   private final double NEGRANGE_MAX_ACCEL = 0.03;//0.02;//0.03;
   private final double MAX_DECEL = 0.03;//0.04;//0.06;

   
//    private double previousLeftSpeed = 0;
//    private double previousRightSpeed = 0;
    private double previousThrottle = 0;

   //tunable
   private final double matrixLength = 2;
   private final double AmatrixLength = 1;

   //not tunable
   private final double weight = 1 / (matrixLength + 1);
   private final double Aweight = 1 / (AmatrixLength + 1);
   private double[] throttleArray = new double[(int)matrixLength];
   private double[] arcadeArray = new double[(int)AmatrixLength];
   private boolean currentInit = true;

   private enum SIDE {LEFT, RIGHT};

   /*
    * no more tuneable
    * things-------------------------------------------------------->>>>>>>>>>>>>>>
    * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */

   public ArcadeDriveController(DriveHardware hardware, XboxController player1) {
       super(hardware, player1);
       
       this.player1 = player1;
       this.hardware = hardware;
       pidDerivative = new TorDerivative(dt);
   }

   @Override
   public void init() {

   }

   @Override
   public void run() {       
        //System.out.println("Stickx: " + player1.getRawAxis(1) + "   sticky: " + player1.getRawAxis(0));

       throttle = -player1.getRawAxis(1);
       double sign = Math.signum(throttle);
       throttle = sign * Math.pow(throttle, 2);

       steer = player1.getRawAxis(0);
       sign = Math.signum(steer);
       steer = sign * Math.pow(steer, 2) * STEER_SCALAR;  

       if(Math.abs(throttle) < 0.025f) {
            throttle = 0;
       }

       if(Math.abs(steer) < 0.025f) {
           steer = 0;
       }

       double rightspeed = 0, leftSpeed = 0;

       if (throttle > previousThrottle) {
           if (previousThrottle> 0)
                throttle = previousThrottle + POSRANGE_MAX_ACCEL;
            else{                
                throttle = previousThrottle + NEGRANGE_MAX_ACCEL;
                //System.out.println("Throttle " + throttle);
            }
                
        }

        if (throttle < previousThrottle && Math.abs(throttle - previousThrottle) > MAX_DECEL) {
            throttle = previousThrottle - MAX_DECEL;
        }

        if(throttle > 1) {
            throttle = 1;
        }

        if(throttle < -1) {
            throttle = -1;
        }


        previousThrottle = throttle;
        
        throttle = -throttle;

       if(throttle > 0) {
           if(steer > 0) {
               leftSpeed = throttle - steer;
               rightspeed = Math.max(throttle, steer);
           } else {
               leftSpeed = Math.max(throttle, -steer);
               rightspeed = throttle + steer;
           }
       } else {
           if(steer > 0) {
               leftSpeed = -Math.max(-throttle, steer);
               rightspeed = throttle + steer;
           } else {
               leftSpeed = throttle - steer;
               rightspeed = -Math.max(-throttle, -steer);
           }
       }

       //hardware.setMotorSpeeds(-leftSpeed, -rightspeed); 
       //System.out.println("Left Speed: " + hardware.getLeftVelocity());
       //System.out.println("Right Speed: " + hardware.getRightVelocity());
       //System.out.println("Left target: " + leftSpeed * MAX_VELOCITY);
       //System.out.println("Right target: " + rightspeed * MAX_VELOCITY);

       //MAX SPEED 16,000 TICKS PER SECOND

       //convert to requested speed in encoder ticks per second
       //convertedspeed = max speed * leftSpeed

        // if(leftSpeed > previousLeftSpeed) {
        //    leftSpeed = previousLeftSpeed + MAX_ACCEL;
        // }
        // if(leftSpeed < previousLeftSpeed) {
        //     leftSpeed = previousLeftSpeed - MAX_DECEL;
        // }

        // if(rightspeed > previousRightSpeed) {
        //     rightspeed = previousRightSpeed + MAX_ACCEL;
        // }
        // if(rightspeed < previousRightSpeed) {
        //     rightspeed = previousRightSpeed - MAX_DECEL;
        // }

        

        // previousLeftSpeed = leftSpeed;
        // previousRightSpeed = rightspeed;

        

        double settingLeftSpeed = leftSpeed, settingRightSpeed = rightspeed;

        if(Math.abs(settingLeftSpeed) < 0.05) {
            settingLeftSpeed = 0;
        }

        if(Math.abs(settingRightSpeed) < 0.05) {
            settingRightSpeed = 0;
        }

       leftOutput = PID(hardware.getLeftVelocity(), settingLeftSpeed * MAX_VELOCITY, SIDE.LEFT);
       rightOutput = PID(hardware.getRightVelocity(), settingRightSpeed * MAX_VELOCITY, SIDE.RIGHT);

       if (Math.abs(leftOutput) < 0.01 && Math.abs(rightOutput) < 0.01) 
            hardware.setMotorSpeeds(0, 0);
       else 
            hardware.setMotorSpeeds(-leftOutput, -rightOutput);

       //System.out.println("Left output: " +  -leftOutput);
       //System.out.println("Right output: " + -rightOutput);
       //System.out.println("Current error: " + currentError);
   }

   public void testRun(XboxController player1){
       double leftSpeed, rightSpeed = 0;
       if(player1.getYButton()){
           leftSpeed = 0.4;
           rightSpeed = 0.4;
        //    leftOutput = PID(hardware.getLeftEncoder(), leftSpeed);
        //    rightOutput = PID(hardware.getRightEncoder(), rightSpeed);
           hardware.setMotorSpeeds(-leftOutput, -rightOutput);
       }
       else if(player1.getAButton()){
            leftSpeed = -0.4;
            rightSpeed = -0.4;
            //leftOutput = PID(hardware.getLeftEncoder(), leftSpeed);
            //rightOutput = PID(hardware.getRightEncoder(), rightSpeed);
            hardware.setMotorSpeeds(-leftOutput, -rightOutput);
       }
       else{
            leftSpeed = 0;
            rightSpeed = 0;
            // leftOutput = PID(hardware.getLeftEncoder(), leftSpeed);
            // rightOutput = PID(hardware.getRightEncoder(), rightSpeed);
            hardware.setMotorSpeeds(-leftOutput, -rightOutput);
       }

   }

   @Override
   public double getLeftOutput() {
       return leftOutput;
   }

   @Override
   public void setLeftOutput(double left) {
       leftOutput = left;
   }

   @Override
   public double getRightOutput() {
       return rightOutput;
   }

   @Override
   public void setRightOutput(double right) {
       rightOutput = right;
   }

   @Override
   public void limeLightTop(boolean top) {
    //    limeLightTop = top;
   }

   public void setTargets(double velocity, double omega) {
       //sets leftMotorSpeed and rightMotorSpeed

   }

   public double powerToRPM(double power) {
       return power * 5500; //replace 5500 w/ the actual maximum speed (run a test)
   }

   public double RPMtoPower(double rpm) {
       return rpm / 5500; //same thing here
   }

   public double PID(double currentSpeed, double targetSpeed, SIDE side) {
        
        currentError = targetSpeed - currentSpeed;

        // SmartDashboard.putNumber("currentError:", currentError);
        pidDerivativeResult = pidDerivative.estimate(currentError);
        pidIntegral += currentError;

        if(currentError < 20) {
        pidIntegral = 0;
        }

        if(pidIntegral * velocitykI > 0.5) {
        pidIntegral = 0.5 / velocitykI;
        } else if(pidIntegral * velocitykI < -0.5) {
        pidIntegral = -0.5 / velocitykI;
        }

        if(side == SIDE.LEFT) {
            LeftPIDSum = ((currentError * velocitykP) +
            (pidIntegral * velocitykI) +
            (pidDerivativeResult * velocitykD)); //+ FeedForward;
    
            return LeftPIDSum;
        } else if(side == SIDE.RIGHT) {
            RightPIDSum = ((currentError * velocitykP) +
            (pidIntegral * velocitykI) +
            (pidDerivativeResult * velocitykD)); //+ FeedForward;
    
            return RightPIDSum;
        } else {
            return 0;            
        }


   }

   public double function(double x) {
       if(x >= 0) {
           return (0.5) * (1 - Math.cos(Math.PI * (Math.pow(x, 1.75))));
       } else {
           return -(0.5) * (1 - Math.cos(Math.PI * (Math.pow(Math.abs(x), 1.75))));
       }
   }

   public void disable() {

   }
}
