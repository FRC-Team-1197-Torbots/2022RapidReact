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

    private DigitalInput bottomBeam;
    private DigitalInput shooterBeam;
    private DigitalInput topBeam;

    private boolean prevTopBeam, prevShooterBeam, prevBottomBeam;

    private CANSparkMax elMotorTop;
    private CANSparkMax elMotorBottom;

    private boolean ballInElevator = false;
    public static int ballcount;
    
    private double PrevTime;

    private Flywheel flywheel;

    public Elevator(Flywheel flywheel) {
        this.flywheel = flywheel;
        bottomBeam = new DigitalInput(0);
        prevBottomBeam = bottomBeam.get();

        topBeam = new DigitalInput(3);
        prevTopBeam = topBeam.get();

        shooterBeam = new DigitalInput(2);
        prevShooterBeam = shooterBeam.get();

        elMotorTop = new CANSparkMax(10, MotorType.kBrushless);
        elMotorBottom = new CANSparkMax(5, MotorType.kBrushless);

        ballcount = 0;
    }

    public void init() {
        PrevTime = Timer.getFPGATimestamp() + 0.2f;
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public static enum autoElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        testBreakBeam();

        switch(elevatorState) {
            case IDLE:
                
                if (ballcount == 0) {
                    elMotorTop.set(0.5);
                    elMotorBottom.set(0.5);
                    if (topBeam.get() && !prevTopBeam) {
                        ballcount++;
                    }
                }
                else if (ballcount == 1) {
                    elMotorTop.set(0);
                    elMotorBottom.set(0.5);
                    if (bottomBeam.get() && !prevBottomBeam) {
                        ballcount++;
                    }
                }
                else if (ballcount == 2) {
                    elMotorTop.set(0);
                    elMotorBottom.set(0);
                }

                prevTopBeam = topBeam.get();
                prevBottomBeam = bottomBeam.get();
                
            break;

            case STORE:
                if (ballcount == 0) {
                    elMotorTop.set(0.5);
                    elMotorBottom.set(0.5);
                    if (topBeam.get() && !prevTopBeam) {
                        ballcount++;
                    }
                }
                else if (ballcount == 1) {
                    elMotorTop.set(0);
                    elMotorBottom.set(0.5);
                    if (bottomBeam.get() && !prevBottomBeam) {
                        ballcount++;
                    }
                }
                else if (ballcount == 2) {
                    elMotorTop.set(0);
                    elMotorBottom.set(0);
                }

                prevTopBeam = topBeam.get();
                prevBottomBeam = bottomBeam.get();
            break;

            case SHOOT:
                    
                    if (flywheel.OnTarget) {
                        elMotorTop.set(0.6);
                        elMotorBottom.set(0.6);
                        if (ballcount > 0) {
                            if (!topBeam.get() && prevTopBeam) {
                                ballcount--;
                                flywheel.setPIDValues(2);
                            }
                        }
                        else if (ballcount == 0) {
                            //nuffin...
                            flywheel.setPIDValues(1);
                        }
                        
                    }
                    else {
                        elMotorTop.set(0);
                        elMotorBottom.set(0);
                    }

                    if (bottomBeam.get() && !prevBottomBeam) {
                        ballcount++;
                    }

                    prevBottomBeam = bottomBeam.get();
                    prevTopBeam = topBeam.get();

                    
                           
            break;
            
        }
    }
    /*
    public void autoRun(autoElevator autoElState) {
        switch (autoElState) {
            case IDLE: 
                if(ballcount < 2) {    
                    elMotor.set(0.2f);

                    if (bottomBeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = bottomBeam.get();                    
                } else if(ballcount == 2) {
                    elMotor.set(0);
                }

                PrevTime = Timer.getFPGATimestamp();
        }
    }
    */


    public void testBreakBeam() {
        SmartDashboard.putBoolean("bottomBeam", bottomBeam.get()); 
        SmartDashboard.putBoolean("topBeam", topBeam.get());
        SmartDashboard.putBoolean("Shooter Beam", shooterBeam.get());
        SmartDashboard.putNumber("Balls in robot", ballcount);       
    }
    
    public boolean isBallInElevator() {
        return ballInElevator;
    }

    public void resetBallCount() {
        ballcount = 0;
    }

    public void setBallCount(int count) {
        ballcount = count;
    }
}