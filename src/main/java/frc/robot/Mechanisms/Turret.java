package frc.robot.Mechanisms;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PID_Tools.TorDerivative;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DigitalInput;
//DIO 0

//import edu.wpi.first.wpilibj.motorcontrol.Talon;

/*------------------------------------
TURRET CLASS ALLOWS DRIVER TO HOLD DOWN ONE BUTTON TO SHOOT.
CONTROLS 3 THINGS:
    1.LINE UP ROBOT
    2.START/STOP FLYWHEEL
    3.START/STOP ELEVATOR

CALLS METHODS FROM ELEVATOR, FLYWHEEL, AND LIMELIGHTLINEUP CLASS.


NOTE margin of error for shooting from 20ft is +- 5 degrees
    MOE from safe zone @ ~190inches is +- 7 degrees
------------------------------------*/

//import edu.wpi.first.wpilibj.Encoder;

//private Encoder turretEncoder;

//public Turret(){
    //turretEncoder = new Encoder()
//}

public class Turret {
    public static enum runTurret{
        INIT, IDLE, LINEUP, REVUP, SHOOT;
    }

    public enum INIT_STATES {
        INIT,TURN, RETURN, ZERO, IDLE
    }

    private INIT_STATES m_initstate;
    
    private runTurret turretState = runTurret.IDLE;
    public TalonSRX TurretMotor;
    private DigitalInput zeroSensor;
    //private XboxController player2;
    private LimeLightLineup limelight;

    private double currenterror;
    private double units;
    private double small_rotations;
    private double degrees;
    private double big_rotations;
    private double horizAngleOffset;
    private final double gearRatio = 18f/220f;//1.26 / 41.625;
    private int offTargetCount = 0;

    /** PID VARs */
    private double TargetAngle; //in degrees
    private final double dt = 0.005f;
    private TorDerivative TurretDerivative;
    private double pidIntegral = 0;
    private final double turretKP = 0.015f;
    private final double turretKI = 0.00000f;
    private final double turretKD = 0.0000f;
    private final double OnTargetDelta = 0.25f;
    private double kFF = 0.06f;//0.06f;
    private boolean OnTarget;

    public Turret(){
        TurretMotor = new TalonSRX(8);
        zeroSensor = new DigitalInput(1);
        limelight = new LimeLightLineup();

        m_initstate = INIT_STATES.INIT;

        //PID
        TurretDerivative = new TorDerivative(dt);

        //hard coding target for now
        TargetAngle = 0;
        pidIntegral = 0;
        OnTarget = false;
    }

    /**
     * Function to initially zero the robot
     * turret
     */
    public void init(){
        TurretMotor.setNeutralMode(NeutralMode.Brake);
        TurretMotor.setSelectedSensorPosition(0);
        
        /*switch(m_initstate) {
            case INIT:
                TurretMotor.setSelectedSensorPosition(0);
                m_initstate = INIT_STATES.TURN;
            break;

            case TURN:
                //turn 45 degress                
                if(Math.abs(units_to_degrees(TurretMotor.getSelectedSensorPosition())) >= 45.0f ) {
                    TurretMotor.set(ControlMode.PercentOutput, 0.0f);
                    m_initstate = INIT_STATES.RETURN;
                } else {
                    TurretMotor.set(ControlMode.PercentOutput, -0.7f);
                }
            break;

            case RETURN:
                //return to the zero position
                // System.out.println(zeroSensor.get());
                if(!zeroSensor.get()) {
                    m_initstate = INIT_STATES.ZERO;
                    TurretMotor.set(ControlMode.PercentOutput, 0.0f);
                } else {
                    TurretMotor.set(ControlMode.PercentOutput, 0.4f);
                }
            break;

            case ZERO:
                
                //set to zero
                TurretMotor.setSelectedSensorPosition(0);
                m_initstate = INIT_STATES.IDLE;
            break;

            case IDLE:

            break;            
        }*/

    }

    //conversions helper methods
    private double degrees_to_units(double degrees){
        big_rotations = degrees / 360f;
        small_rotations = big_rotations / gearRatio;
        units = small_rotations * 4096f;
        return units;
    }

    private double units_to_degrees(double units){
        small_rotations = units / 4096f;
        big_rotations = small_rotations * gearRatio;
        degrees = big_rotations * 360f;
        return degrees;
    }

    private boolean offTargetForALongTime(double tx) {
        if (tx == 0) {
            offTargetCount++;
        }
        else
            offTargetCount = 0;
        if (offTargetCount > 10) {
            return true;
        }
        else
            return false;
    }

    public void PIDAutoTuning(double angle) {    
        
        TargetAngle = angle;

        if (TargetAngle > 180)
            TargetAngle = 180;
        else if (TargetAngle <-155)
            TargetAngle = -155;

            //start writing state machine to turn for tuning
        double pidout = TurretPID(units_to_degrees(TurretMotor.getSelectedSensorPosition()), TargetAngle);
            // System.out.println("target " + TargetAngle);
            
        TurretMotor.set(ControlMode.PercentOutput, pidout);   

        /*if(m_initstate != INIT_STATES.IDLE) {
            init();
        } else {
            
                
        }*/
    }
    
    public void PIDTuning(double tx) {      
        if (offTargetForALongTime(tx))
               TargetAngle = 0;
        else
            TargetAngle = units_to_degrees(TurretMotor.getSelectedSensorPosition()) + tx;
            //System.out.println(TargetAngle);


        if (TargetAngle > 180)
            TargetAngle = 180;
        else if (TargetAngle <-155)
            TargetAngle = -155;

            //start writing state machine to turn for tuning
        double pidout = TurretPID(units_to_degrees(TurretMotor.getSelectedSensorPosition()), TargetAngle);
            // System.out.println("target " + TargetAngle);
            
        TurretMotor.set(ControlMode.PercentOutput, pidout);            
    }

    private double TurretPID(double currentangle, double targetangle) {
        currenterror = currentangle - targetangle;

        double pidDerivativeResult = TurretDerivative.estimate(currenterror);
        pidIntegral += currenterror;


        if(currenterror < 20) {
            pidIntegral = 0;
        }

        if(pidIntegral * turretKI > 0.5) {
            pidIntegral = 0.5 / turretKI;
        } else if(pidIntegral * turretKI < -0.5) {
            pidIntegral = -0.5 / turretKI;
        }

        // System.out.println(currenterror);

        return ((currenterror * turretKP) +
        (pidIntegral * turretKI) +
        (pidDerivativeResult * turretKD) + (kFF * Integer.signum((int)currenterror))); //+ FeedForward;
                                            //if currenterror is positive, kFF is positive.  if currenterror is negative, kFF is negative
    }

    public boolean isDone(){
        if(Math.abs(currenterror) <= 1.0f){
            return true;
        }
        else{
            return false;
        }
    }

    /*
    public void manualControl(String direction){
        //false is when the sensor is on, and true is when the sensor is off
        if(!(zeroSensor.get())){
            TurretMotor.setSelectedSensorPosition(0);
        }

        //System.out.println(breakBeam.get());
        //Diameter for large circle is 41.625 inches.
        //Diameter for inner circle is 1.26 inches.
        if(direction.equals("right")){
            TurretMotor.set(ControlMode.PercentOutput, 0.2f);
            units = TurretMotor.getSelectedSensorPosition();
            units_to_degrees(units);
            System.out.println("Degrees " + degrees);
        }
        else if(direction.equals("left")){
            TurretMotor.set(ControlMode.PercentOutput, -0.2f);
            units = TurretMotor.getSelectedSensorPosition();
            units_to_degrees(units);
            System.out.println("Degrees " + degrees);
        }
        else if(direction.equals("stop")){
            TurretMotor.set(ControlMode.PercentOutput, 0);
            units = TurretMotor.getSelectedSensorPosition();
            units_to_degrees(units);
            System.out.println("Degrees " + degrees);
        }
    }
    */
    public void manualControl(double speed) {
        if (Math.abs(units_to_degrees(TurretMotor.getSelectedSensorPosition())) > 120)
            TurretMotor.set(ControlMode.PercentOutput, 0);
        else  
            TurretMotor.set(ControlMode.PercentOutput, speed);
    }

    public void manualZero() {
        if (!zeroSensor.get()) {
            TurretMotor.set(ControlMode.PercentOutput, 0);
            TurretMotor.setSelectedSensorPosition(0);
        }
        else {
            if (units_to_degrees(TurretMotor.getSelectedSensorPosition()) > 0) {
                TurretMotor.set(ControlMode.PercentOutput, 0.2f);
            }
            else {
                TurretMotor.set(ControlMode.PercentOutput, -0.2f);
            }
        }

        
    }

    public void onDisable(){
        TurretMotor.setNeutralMode(NeutralMode.Coast);
    }

}


