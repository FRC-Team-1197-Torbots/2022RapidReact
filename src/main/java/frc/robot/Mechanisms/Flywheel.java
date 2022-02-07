package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE FLYWHEEL SPEEDS
----------------------*/

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class Flywheel {

    private Joystick player2;
    private CANSparkMax motor1;
    private CANSparkMax motor2;


    public Flywheel(XboxController player22) {
        this.player2 = player2;
        motor1 = new CANSparkMax(1, MotorType.kBrushless);
        motor2 = new CANSparkMax(2, MotorType.kBrushless);
    }



    public void init(){

    }

    public void revUp(double distance){
        //Calculate targetSpeed through distance ranges
        //use PID to constantly increase the speed to a target value (probably reuse Brennan's code)

        motor1.set(0.5);
        motor2.set(0.5);

    }

    public void stop(){
        //set motor speed to 0
        motor1.set(0);
        motor2.set(0);
    }
    
}
