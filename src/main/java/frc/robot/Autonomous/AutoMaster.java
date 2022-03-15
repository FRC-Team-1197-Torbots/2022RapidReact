package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.DriveHardware;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.MechMaster;

public class AutoMaster {

    private static final String Auto_2Ball = "2Ball";
    private static final String Auto_4Ball = "4Ball";
    private static final String Auto_1Ball = "1Ball";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private AutoBall1 auto1;
    private AutoBall2 auto2;

    private MechMaster mechMaster;
    private TorDrive drive;

    public AutoMaster (TorDrive drive, MechMaster mechMaster) {
        this.drive = drive;
        this.mechMaster = mechMaster;
        auto1 = new AutoBall1(mechMaster, drive);
        auto2 = new AutoBall2(mechMaster, drive);

    }
    
    public void robotInit() {
        m_chooser.setDefaultOption("4 ball", Auto_4Ball);
        m_chooser.addOption("2 ball", Auto_2Ball);
        m_chooser.addOption("1 ball", Auto_1Ball);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void init() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }


    public void run() {
        switch (m_autoSelected) {
            case Auto_4Ball:
              // Put custom auto code here
              break;
            case Auto_2Ball:
              auto2.run();
            
              break;
            case Auto_1Ball:
              auto1.run();
              break;
        }
    }

    /***************************************
    RUN THIS MULTIPLE TIMES W/ 0 FOR NUMTICKS, PUSH IT 1 FOOT EVERY TIME
    TAKE AVERAGE OF EVERY FINAL ENCODER VALUE IT PRINTS
    PLUG THAT VALUE BACK INTO NUMTICKS, SEE IF IT GOES 1 FOOT OR NOT
        IF IT DOES --> PID ISSUE
        IF IT DOESN'T --> ???
    **POSSIBLY GO MORE THAN 1 FOOT TO GET CLEARER RESULTS..
    ***************************************/
    public void testRun(int numTicks) {
        int ticks = numTicks;
        double currentTicks = drive.getAverageEncoderPosition();
        if (currentTicks < ticks)
            drive.setMotorSpeeds(0.05, 0.05);
        else {
            drive.setMotorSpeeds(0, 0);
            System.out.println("Current ticks: " + currentTicks);
        }
    }
}
