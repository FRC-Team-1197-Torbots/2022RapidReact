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
        private NetworkTableInstance networkTableInstance;
        private NetworkTable table;// for limelight
        private NetworkTableEntry tx;
        private NetworkTableEntry ty;
        private NetworkTableEntry ta;
        private NetworkTableEntry tv;
        private NetworkTableEntry ts;
        private double x;
        private double y;
        private double area;
        private double v;
        private double s;
        

        public LimeLightLineup(){
                table = NetworkTableInstance.getDefault().getTable("limelight");
                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                tv = table.getEntry("tv");
                ts = table.getEntry("ts");

                


                
        }
        
        public void init(){

        }

        public void test(){
                
                NetworkTableInstance.getDefault().getTable("limelight");


                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                tv = table.getEntry("tv");
                ts = table.getEntry("ts");

                // x = tx.getNumber(defaultValue)
                y = ty.getDouble(0.0);
                area = ta.getDouble(0.0);
                v = tv.getDouble(0.5);


               // System.out.println("X: " + x);
                //System.out.println("Y: " + y);
                //System.out.println("Area: " + area);
                
                SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
                SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
                SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
                SmartDashboard.putNumber("V_Number", tv.getDouble(0.0));
                SmartDashboard.putNumber("S_number", ts.getDouble(0.0));
                
        }
    
        public void lineup(){
                
                float Kp = -0.1f;
                float min_command = 0.05f;

                x = tx.getDouble(0.0);
                y = ty.getDouble(0.0);

                /*
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
                */
        }

        public double getAngle() {
                x = tx.getDouble(0.0);
                return x;
        }
        public void calculate_distance() {
                //IF isAligned = true
                //use the function d = (h2-h1) / tan(a1+a2) to calculate
        }
    
}
