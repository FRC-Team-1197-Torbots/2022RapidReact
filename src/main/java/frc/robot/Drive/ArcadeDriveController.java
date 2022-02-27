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
   private BantorPID limeLightPID;
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

   // limelight PID
   // Stuff------------------------------>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

   private final double positionkP = -2; //-1.4
   private final double positionkI = 0; //-0.002
   private final double positionkD = -0.035; //-0.035
   private final double positionTolerance = 3 * Math.PI / 180.0;// for thePID
   private final double velocitykP = 0.0;// velocity stuff probably not needed at all and should keep 0
   private final double velocitykI = 0.0;
   private final double velocitykD = 0.0;
   private final double kV = 0.0;// this should definitely stay at 0
   private final double kA = 0.0;// this should definitely stay at 0
   private final double velocityTolerance = 0.0;
   private final double targetVelocity = 0.0;// probably won't need
   private final double targetAcceleration = 0.0;// probably won't need

   // ------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

   //this is for the curve drive

   //tunable
   private final double matrixLength = 2;
   private final double AmatrixLength = 1;

   //not tunable
   private final double weight = 1 / (matrixLength + 1);
   private final double Aweight = 1 / (AmatrixLength + 1);
   private double[] throttleArray = new double[(int)matrixLength];
   private double[] arcadeArray = new double[(int)AmatrixLength];
   private boolean currentInit = true;

   /*
    * no more tuneable
    * things-------------------------------------------------------->>>>>>>>>>>>>>>
    * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */

   public ArcadeDriveController(DriveHardware hardware, XboxController player1) {
       super(hardware, player1);
       
       this.player1 = player1;
       this.hardware = hardware;
   }

   @Override
   public void init() {

   }

   @Override
   public void run() {
       double throttle = player1.getRawAxis(1);
       double steer = player1.getRawAxis(0);

       double sign = 0;

       if(steer > 0) {
        sign = 1;
       } else if(steer < 0) {
        sign = -1;
       }

       if(Math.abs(steer) < 0.25f) {
           steer = 0;
       } else {
           steer = Math.pow(steer, 2) * sign;
       }
       
       double rightspeed = 0, leftSpeed = 0;

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



       hardware.setMotorSpeeds(leftSpeed, rightspeed);
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

   public double PID(double currentSpeed, double targetSpeed) {
        /*
        error = targetSpeed - currentSpeed;

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
        */

        return 0.0;

   }

   public double function(double x) {
       if(x >= 0) {
           return (0.5) * (1 - Math.cos(Math.PI * (Math.pow(x, 1.75))));
       } else {
           return -(0.5) * (1 - Math.cos(Math.PI * (Math.pow(Math.abs(x), 1.75))));
       }
   }
}
