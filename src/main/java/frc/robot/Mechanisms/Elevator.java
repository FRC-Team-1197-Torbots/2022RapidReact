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

    private boolean ballInElevator = false;
    public int ballcount;
    private boolean prev, shooterprev;
    private double PrevTime;

    private Flywheel flywheel;

    public Elevator(Flywheel flywheel) {
        this.flywheel = flywheel;
        breakbeam = new DigitalInput(0);
        ShooterBeam = new DigitalInput(2);
        elMotor = new CANSparkMax(10, MotorType.kBrushless);
        prev = breakbeam.get();
        shooterprev = ShooterBeam.get();
        ballcount = 0;
    }

    public void init() {
        PrevTime = Timer.getFPGATimestamp() + 0.2f;
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        testBreakbeam();

        switch(elevatorState) {
            case IDLE:
                if(Timer.getFPGATimestamp() < PrevTime + 0.1f) {                    
                    elMotor.set(-0.6);                    
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
                PrevTime = Timer.getFPGATimestamp();                
            break;
            case SHOOT:
                elMotor.set(0.6);
                
                if(ShooterBeam.get() && !shooterprev) {
                    ballcount--;

                    // SWITCHES PID VALUES
                    if (ballcount == 1) {
                        flywheel.setPIDValues(2);
                    }
                    else if(ballcount == 0) {
                        flywheel.pidIntegral = 0;
                        flywheel.setPIDValues(1);
                    }
                }
                
                PrevTime = Timer.getFPGATimestamp();
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

    public void resetBallCount() {
        ballcount = 0;
    }
}