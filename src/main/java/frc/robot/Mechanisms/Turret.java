package frc.robot.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
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
    private final TalonSRX TurretMotor;
    private DigitalInput zeroSensor;
    private DigitalInput breakBeam;
    private XboxController player2;

    private double units;
    private double small_rotations;
    private double degrees;
    private double big_rotations;
    private final double gearRatio = 18f/220f;//1.26 / 41.625;


    /** PID VARs */
    private double TargetAngle; //in degrees
    private final double dt = 0.005f;
    private TorDerivative TurretDerivative;
    private double pidIntegral = 0;
    private final double turretKP = 0.009;
    private final double turretKI = 0.00000f;
    private final double turretKD = 0.0000f;
    private final double OnTargetDelta = 0.25f;
    private double kFF = 0.06f;
    private Boolean OnTarget;

    public Turret(TalonSRX talon, XboxController player2){
        this.TurretMotor = talon;
        this.player2 = player2;
        zeroSensor = new DigitalInput(0);
        breakBeam = new DigitalInput(1);
        talon = new TalonSRX(10);


        m_initstate = INIT_STATES.TURN;

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
        
        switch(m_initstate) {
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
                    TurretMotor.set(ControlMode.PercentOutput, -0.2f);
                }
            break;

            case RETURN:
                //return to the zero position
                if(!zeroSensor.get()) {
                    m_initstate = INIT_STATES.ZERO;
                    TurretMotor.set(ControlMode.PercentOutput, 0.0f);
                } else {
                    TurretMotor.set(ControlMode.PercentOutput, 0.2f);
                }
            break;

            case ZERO:
                //set to zero
                TurretMotor.setSelectedSensorPosition(0);
                m_initstate = INIT_STATES.IDLE;
                //System.out.println("Zero");
            break;

            case IDLE:

            break;
        }
        /*units = degrees_to_units(45f);
        talon.set(ControlMode.PercentOutput, 0.2f);
        talon.setSelectedSensorPosition(units);
        //talon.set(ControlMode.PercentOutput, 0);
        talon.set(ControlMode.PercentOutput, -0.2f);
        if(!(zeroSensor.get())){
            talon.set(ControlMode.PercentOutput, 0);
            talon.setSelectedSensorPosition(0);
            System.out.println("zeroed.");
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

    
    public void PIDTuning(double stick) {
        TargetAngle += stick * 5;

        if(m_initstate != INIT_STATES.IDLE) {
            init();
        } else {
            //start writing state machine to turn for tuning
            double pidout = TurretPID(units_to_degrees(TurretMotor.getSelectedSensorPosition()), TargetAngle);
            // System.out.println("target " + TargetAngle);
            
            TurretMotor.set(ControlMode.PercentOutput, pidout);            
        }
    }

    private double TurretPID(double currentangle, double targetangle) {
        double currenterror = currentangle - targetangle;

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
        (pidDerivativeResult * turretKD) + (kFF * (currenterror/Math.abs(currenterror)))); //+ FeedForward;
    }


    public void test(String direction){
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

    public void run() {
    /*
    CURRENTLY CODING BASED ON ASSUMPTION THAT ROBOT STOPS BEFORE SHOOTING. NOT CONSIDERING MOVING 
    SHOTS JUST YET
    
        switch(turretState) {
            case INIT:
                if(m_initstate != IDLE) {
                    init();
                } else {
                    turretState = IDLE;
                }
            break;

            case IDLE:
                flywheel.run(false, false, 0);
                elevator.run(runElevator.IDLE);

                if (player2.getXButtonPressed()) {
                    turretState = runTurret.LINEUP;
                }
            
            case LINEUP:
                limelight.lineup();
                if (limelight.linedUp()) {
                    flywheel.init();
                    turretState = runTurret.REVUP;
                }
            case REVUP:
                flywheel.run(true, true, limelight.calculateDistance());
                if (flywheel.isFastEnough());
                    //elevator.init();
                    turretState = runTurret.SHOOT;
            case SHOOT:
                elevator.run(runElevator.RUN);
            
            if (player2.getXButtonReleased()) {
                turretState = runTurret.IDLE;
            }
        }*/ 
    }
}
