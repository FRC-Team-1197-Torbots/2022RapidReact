package frc.robot.Mechanisms;

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

    public void run() {
        switch(intakeState) {
            case UP:
                //set motor speed to 0
                //if button pressed
                    //intakeState = moveIntake.GOING_DOWN
            case GOING_DOWN:
                //move motor speed x amount
                //if going down is done
                    //intakeState = moveIntake.DOWN
            case DOWN:
                //set roller speed to x amount
                //if button pressed
                    //intakeState = moveIntake.GOING_UP
            case GOING_UP:
                //set motor speed x amount
                //if going up is done
                    //intakeState = moveIntake.UP
        }

    }
    
    
}
