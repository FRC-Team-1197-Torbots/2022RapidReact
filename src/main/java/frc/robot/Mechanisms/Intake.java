package frc.robot.Mechanisms;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;

/*-------------------------
INTAKE CLASS CONTROLS EVERYTHING INTAKE RELATED. DRIVER PRESSES 1 BUTTON TO TOGGLE INTAKE
    -DROPS AND RAISES INTAKE
    -STARTS/STOPS ROLLERS
**RUNS SEPARATELY FROM THE TURRET CLASS
-------------------------*/

public class Intake {
    
    public static enum moveIntake{
        UP, DOWN, GOING_UP, GOING_DOWN;
    }

    private moveIntake intakeState = moveIntake.UP;

    public void run(moveIntake intakeState) {
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
    
    
}
