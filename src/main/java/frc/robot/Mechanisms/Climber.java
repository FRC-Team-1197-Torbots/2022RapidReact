package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*-------------------------
CONTROLS 4 CLIMBER ARMS
-2 BUTTONS TO EXTEND/RETRACT INNER ARMS
-2 BUTTONS TO EXTEND/RETRACT OUTER ARMS
-------------------------*/


public class Climber {

    private CANSparkMax rightClMotor;
    private CANSparkMax leftClMotor;
    private CANSparkMax nikitaClMotor;

    private RelativeEncoder rightClEncoder;
    private RelativeEncoder leftClEncoder;

    private double speed = 0.8;
    private double nikitaSpeed = 0.3;

    public enum climbState{
        UP, DOWN, IDLE;
    }

    public enum nikitaState {
        UP, DOWN, IDLE;
    }
    
    //public climbState climberPos;

    public Climber(){
        rightClMotor = new CANSparkMax(9, MotorType.kBrushless);
        leftClMotor = new CANSparkMax(6, MotorType.kBrushless);
        //nikitaClMotor = new CANSparkMax(7, MotorType.kBrushless);

        rightClEncoder = rightClMotor.getEncoder();
        leftClEncoder = leftClMotor.getEncoder();

        rightClEncoder.setPosition(0);
        leftClEncoder.setPosition(0);
        


    }

    public void climb(climbState climberPos){
        SmartDashboard.putNumber("Right Climber", getRightClimberPos());
        SmartDashboard.putNumber("Left Climber", getLeftClimberPos());

        switch(climberPos) {
            case UP:
                rightClMotor.set(-speed);
                leftClMotor.set(speed);
                break;
            case DOWN:
                rightClMotor.set(speed);
                leftClMotor.set(-speed);
                break;
            case IDLE:
                rightClMotor.set(0);
                leftClMotor.set(0);
                break;

        }
    }

    public double getLeftClimberPos() {
        return leftClEncoder.getPosition();
    }

    public double getRightClimberPos() {
        return rightClEncoder.getPosition();
    }

    public boolean isAboveZero() {
        return getLeftClimberPos() > 0 && getRightClimberPos() < 0;
    }
    /*
    public void nikita(nikitaState nikitaPos) {
        switch(nikitaPos) {
            case UP:
                nikitaClMotor.set(nikitaSpeed);
                break;
            case DOWN:
                nikitaClMotor.set(-nikitaSpeed);
                break;
            case IDLE:
                nikitaClMotor.set(0);
                break;
        }
    }

    public double testNikita() {
        return nikitaClMotor.get();
    }
    */

    
}
