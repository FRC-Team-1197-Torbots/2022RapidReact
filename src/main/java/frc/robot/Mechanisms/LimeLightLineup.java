package frc.robot.Mechanisms;
/*--------------------------------
LIMELIGHTLINEUP CLASS ALIGNS THE TURRET WITH THE CENTER HUB.
ONCE IT'S ALIGNED, IT CAN ALSO CALCULATE DISTANCE.
--------------------------------*/

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLightLineup {
        //private NetworkTableInstance networkTableInstance;
        //private final NetworkTable table;// for limelight
       // private final NetworkTableEntry tx;
       // private final NetworkTableEntry ty;
        //private final NetworkTableEntry ta;
        private double x;
        private double y;
/*
        public LimeLightLineup(){
                table = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
        }
        
        public void init(){

        }
    /*
        public void lineup(){
                float Kp = -0.1f;
                float min_command = 0.05f;

                std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
                float tx = table->GetNumber("tx");

                if (joystick->GetRawButton(9))
                {
                        float heading_error = -tx;
                        float steering_adjust = 0.0f;
                        if (tx > 1.0)
                        {
                                steering_adjust = Kp*heading_error - min_command;
                        }
                        else if (tx < 1.0)
                        {
                                steering_adjust = Kp*heading_error + min_command;
                        }
                        left_command += steering_adjust;
                        right_command -= steering_adjust;
                }
        }
        public void calculate_distance(){
                //IF isAligned = true
                //use the function d = (h2-h1) / tan(a1+a2) to calculate
        }
    */
    
}
