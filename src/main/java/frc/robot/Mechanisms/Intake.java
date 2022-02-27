package frc.robot.Mechanisms;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Robot;
import frc.robot.Drive.DriveController;
import frc.robot.PID_Tools.TorDerivative;

/*-------------------------
INTAKE CLASS CONTROLS EVERYTHING INTAKE RELATED. DRIVER PRESSES 1 BUTTON TO TOGGLE INTAKE
    -DROPS AND RAISES INTAKE
    -STARTS/STOPS ROLLERS
**RUNS SEPARATELY FROM THE TURRET CLASS
-------------------------*/

public class Intake {
    private CANSparkMax intake, roller;
    private RelativeEncoder intakeEncoder;
    
    private XboxController controller;

    private double rollerin = -0.5f, rollerout = 0.5f;


    //PID Variables
    private double pidDerivativeResult = 0;
    private double pidIntegral = 0;
    private final double kP = 0.05;
    private final double kI = 0;
    private final double kD = 0;
    private boolean ONTARGET;
    private TorDerivative derivative;

    private double target;
    private final double UP_TARGET = 0, DOWN_TARGET = 9.1;

    public Intake(XboxController drivercontroller) {
        intake = new CANSparkMax(4, MotorType.kBrushless);
        roller = new CANSparkMax(11, MotorType.kBrushless);

        intakeEncoder = intake.getEncoder(Type.kHallSensor, 42);
        intakeEncoder.setPosition(0);

        derivative = new TorDerivative(Robot.TIME_INTERVAL);
        derivative.resetValue(0);      
        
        target = UP_TARGET;
        this.controller = drivercontroller;
        ONTARGET = false;
    }
    
    public enum moveIntake{
        UP, DOWN, GOING_UP, GOING_DOWN;
    }

    private moveIntake intakeState = moveIntake.UP;

    public void run() {
        double speed = PID();
        if (ONTARGET)
            intake.set(0);
        else
            intake.set(speed);
        
        switch(intakeState) {
            case UP:
                roller.set(0);
                //no longer on target, set new target, change state
                if(controller.getXButtonPressed()) {
                    ONTARGET = false;
                    target = DOWN_TARGET;
                    intakeState =  moveIntake.DOWN;
                }
            break;

            case DOWN:
                if (ONTARGET)
                    roller.set(rollerin);
                if(controller.getXButtonPressed()) {
                    ONTARGET = false;
                    intakeState =  moveIntake.UP;
                    target = UP_TARGET;
                }
            break;
        }

    }
    
    public double PID() {
        double speed = 0;
        double error = target - intakeEncoder.getPosition();
        System.out.println("position " + intakeEncoder.getPosition());

        pidDerivativeResult = derivative.estimate(error);
        pidIntegral += error;

        if (error <= 0.1)
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

    
}
