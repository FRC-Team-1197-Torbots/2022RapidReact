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
    private RelativeEncoder intakeencoder;

    private TorDerivative derivative;
    private XboxController controller;

    private double rollerin = -0.5f, rollerout = 0.5f;


    //PID Variables
    private double pidDerivativeResult = 0;
    private double pidIntegral = 0;
    private final double kP = 0.4;
    private final double kI = 0;
    private final double kD = 0;
    private boolean ONTARGET;

    private double target;
    private final double UP_TARGET = 0, DOWN_TARGET = 1.785;

    public Intake(XboxController drivercontroller) {
        intake = new CANSparkMax(4, MotorType.kBrushless);
        roller = new CANSparkMax(11, MotorType.kBrushless);

        intakeencoder = intake.getEncoder(Type.kHallSensor, 42);
        intakeencoder.setPosition(0);

        derivative = new TorDerivative(Robot.TIME_INTERVAL);
        derivative.resetValue(0);      
        
        target = UP_TARGET;
        this.controller = drivercontroller;
        ONTARGET = false;
    }
    
    public static enum moveIntake{
        UP, DOWN, GOING_UP, GOING_DOWN;
    }

    private moveIntake intakeState = moveIntake.UP;

    public void run() {
        double speed = PID();
        // intake.set(speed);

        if(controller.getXButton()) {
            target = UP_TARGET;
        } else if(controller.getAButton()) {
            target = DOWN_TARGET;
        }

        if(controller.getBButton()) {
            roller.set(rollerin);
        } else if(controller.getYButton()) {
            roller.set(rollerout);
        } else {
            roller.set(0);
        }

        switch(intakeState) {
            case UP:
            /*
                set roller speeds to 0
                if (motor position != 0)
                    set motor speed to -x amount;
                else
                    set motor speed to 0;
            */
            case DOWN:
            /*
                if (!magsensor.get) {
                    set motor speed to x amount;
                }
                else 
                    set roller speed to x amount;
                    set motor speed to 0;
            */
        }

    }
    
    public double PID() {
        double speed = 0;
        double error = target - intakeencoder.getPosition();
        System.out.println("Error " + error);

        pidDerivativeResult = derivative.estimate(error);
        pidIntegral += error;
        
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
