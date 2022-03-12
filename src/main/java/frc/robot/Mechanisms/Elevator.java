package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE ELEVATOR SPEEDS
----------------------*/

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    private DigitalInput breakbeam;
    private DigitalInput ShooterBeam;
    private CANSparkMax elMotor;
    private Timer timer;

    private boolean ballInElevator = false;
    public int ballcount;
    private boolean prev, shooterprev;

    public Elevator() {
        breakbeam = new DigitalInput(0);
        ShooterBeam = new DigitalInput(2);
        elMotor = new CANSparkMax(10, MotorType.kBrushless);
        timer = new Timer();
        timer.reset();

        prev = breakbeam.get();
        shooterprev = ShooterBeam.get();
        ballcount = 0;
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        testBreakbeam();

        switch(elevatorState) {
            case IDLE:
                if(ballcount > 0) {
                    if(breakbeam.get()) {
                        elMotor.set(-0.6);
                    } else {
                        elMotor.set(0);
                    }
                } else {
                    elMotor.set(0);
                }
                
                
            break;
            case STORE:
                if(ballcount < 2) {
                    //if break beam (digital input) is broken, set motor speed to 0, else set elevator motor speed to x
                    if (breakbeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = breakbeam.get();
                    
                } 

                elMotor.set(0.4);                
            break;
            case SHOOT:
                elMotor.set(0.6);
                
                if(ShooterBeam.get() && !shooterprev) {
                    ballcount--;
                    Flywheel.pidIntegral = 0;
                }
                
                shooterprev = ShooterBeam.get();
            break;
            
        }
    }

    public void testBreakbeam() {
        SmartDashboard.putBoolean("Breakbeam", breakbeam.get()); 
        SmartDashboard.putBoolean("Shooter Beam", ShooterBeam.get());
        SmartDashboard.putNumber("Balls in robot", ballcount);       
    }
    
    public boolean isBallInElevator() {
        return ballInElevator;
    }
}