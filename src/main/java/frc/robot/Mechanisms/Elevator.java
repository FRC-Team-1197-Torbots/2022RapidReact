package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE ELEVATOR SPEEDS
----------------------*/

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator {

    private DigitalInput breakbeam;
    private CANSparkMax elMotor;

    private boolean ballInElevator = false;

    public Elevator() {
        breakbeam = new DigitalInput(0);
        elMotor = new CANSparkMax(7, MotorType.kBrushless);
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        switch(elevatorState) {
            case IDLE:
                elMotor.set(0);
                //set elevator motor speed to 0
            case STORE:
                //if break beam (digital input) is broken, set motor speed to 0, else set elevator motor speed to x
                if (breakbeam.get()){ //or !breakbeam.get()
                    ballInElevator = true;
                    elMotor.set(0);
                }
                else
                    elMotor.set(0.5);
    
            case SHOOT:
                //set elevator motor speed to x
                elMotor.set(0.5);
                ballInElevator = false;
        }
    }

    public void testBreakbeam() {
        System.out.println("Breakbeam: " + breakbeam.get());
    }
    
    public boolean isBallInElevator() {
        return ballInElevator;
    }
}
