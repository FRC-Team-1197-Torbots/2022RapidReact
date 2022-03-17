package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.PID_Tools.TorDerivative;

/*-------------------------
INTAKE CLASS CONTROLS EVERYTHING INTAKE RELATED. DRIVER PRESSES 1 BUTTON TO TOGGLE INTAKE
    -DROPS AND RAISES INTAKE
    -STARTS/STOPS ROLLERS
**RUNS SEPARATELY FROM THE TURRET CLASS
-------------------------*/

public class Intake {
    private CANSparkMax intake, roller;
    // private RelativeEncoder intakeEncoder;
    private Encoder intakeEncoder;
    
    private XboxController controller;

    private double rollerin = -0.5f, rollerout = 0.5f;


    //PID Variables
    private double pidDerivativeResult = 0;
    private double pidIntegral = 0;
    private final double kP = 0.001;//0.00075;
    private final double kI = 0;
    private final double kD = 0;//0.0000062;
    public boolean ONTARGET;
    private TorDerivative derivative;

    private double target;
    private final double UP_TARGET = 0, DOWN_TARGET = 486;

    public enum moveIntake{
        UP, DOWN;
    }

    public moveIntake intakeState = moveIntake.UP;

    public Intake(XboxController drivercontroller) {
        intake = new CANSparkMax(4, MotorType.kBrushless);
        roller = new CANSparkMax(11, MotorType.kBrushless);

        // intakeEncoder = intake.getEncoder(Type.kHallSensor, 42);
        intakeEncoder = new Encoder(14,15, false, Encoder.EncodingType.k4X);
        intakeEncoder.reset();

        derivative = new TorDerivative(Robot.TIME_INTERVAL);
        derivative.resetValue(0);      
        
        target = UP_TARGET;
        this.controller = drivercontroller;
        ONTARGET = false;
    }   
    

    public void run(moveIntake intakeState) {
        this.intakeState = intakeState;
        double speed = PID();
        intake.set(speed);
        SmartDashboard.putNumber("Intake position", intakeEncoder.get());
        //System.out.println(intakeState);
        //SmartDashboard.putNumber("Target value", target);
        //SmartDashboard.putNumber("Speed", speed);

        
        switch(intakeState) {
            case UP:
                roller.set(rollerin / 3);
                target = UP_TARGET;
            break;

            case DOWN:
                target = DOWN_TARGET;
                if (ONTARGET)
                    roller.set(rollerin);
            break;

        }

    }
    
    public double PID() {
        double speed = 0;
        double error = target - intakeEncoder.get();

        pidDerivativeResult = derivative.estimate(error);
        pidIntegral += error;

        if (Math.abs(error) <= 20)
        {
            ONTARGET = true;
        }
        
        if(error < 20) { //magic number we need to calculate
            pidIntegral = 0;
        }

        if(pidIntegral * kI > 0.5) {
            pidIntegral = 0.5 / kI;
        } else if(pidIntegral * kI < -0.5) {
            pidIntegral = -0.5 / kI;
        }

        speed = (error * kP) + (pidIntegral * kI) + (pidDerivativeResult * kD);
        return speed;
    }

    public void testrun(int state) {
        if (state == 1)
            intake.set(0.25);
        else if (state == 2)
            intake.set(-0.25);
    }

    public double getEncoderPos() {
        return intakeEncoder.get();
    }

    
}
