// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import frc.robot.Drive.DriveHardware;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;
import frc.robot.Mechanisms.Elevator.runElevator;
import frc.robot.Autonomous.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final double TIME_INTERVAL = 0.005f;

  private static final String Auto_2Ball = "2Ball";
  private static final String Auto_4Ball = "4Ball";
  private static final String Auto_1Ball = "1Ball";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController player1;
  private XboxController player2;

  //mechanisms
  private Intake intake;
  private Flywheel flywheel;
  private Turret turret;
  //channel port 0 is where the talon encoder is
  private TalonSRX talon;
  private LimeLightLineup limeLight;
  private Elevator elevator;
  private DriveHardware hardware;
  private TorDrive drive;
  private MechMaster mechMaster;
  private AutoMaster autoMaster;

  private boolean isAligned;
  private double horizAngleOffset; //instance variable to test the accuracy of the turret, get rid of this later
  
  public Robot() {
    player1 = new XboxController(0);
    player2 = new XboxController(1);
    //flywheel = new Flywheel(player2);
    //talon = new TalonSRX(10);

    //intake = new Intake(player1);
    // turret = new Turret();
    // limeLight = new LimeLightLineup();
    // elevator = new Elevator();
    hardware = new DriveHardware();
    drive = new TorDrive(hardware, player1);
    mechMaster = new MechMaster();
    autoMaster = new AutoMaster(drive, mechMaster); 

    CameraServer.startAutomaticCapture();
  }
  
  
  
  
  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    autoMaster.robotInit();

    // limelight = new HttpCamera("limelight", "http://limelight.local:5801");
    // CameraServer.startAutomaticCapture(limelight);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoMaster.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoMaster.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mechMaster.TeleInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("Right encoder", drive.getRightEncoder());
    SmartDashboard.putNumber("Left encoder", drive.getLeftEncoder());
    drive.Run();

    /* **************** INTAKE TEST CODE **************************** */
    //intake.run();
    mechMaster.teleRun();


    /* **************** TURRET TEST CODE ****************************
    //turret.PIDTuning(limeLight.getAngle());
    
    limeLight.test();
    //SmartDashboard.putNumber("horizAngleOffset", horizAngleOffset);

    //Hard turn the robot 70 degrees, then force the "error" to be another 50 degrees.
    //System.out.println("Error: " + limeLight.getAngle());
    turret.PIDTuning(limeLight.getAngle());
    
    if(turret.isDone()){
      SmartDashboard.putNumber("Distance: ", limeLight.calculate_distance());
    }
    */

    // ****************** ELEVATOR TEST CODE *****************
    
    //see when the breakbeam returns true or false
    //elevator.testBreakbeam();
    //drive.Run(true, true);

    //test the STORE & SHOOT state
    /*
    if (player2.getXButtonPressed())
      elevator.run(runElevator.STORE);
    else if (player2.getBButtonPressed())
      elevator.run(runElevator.SHOOT);
    else
      elevator.run(runElevator.IDLE);
    */
    

    /* ***************FLY WHEEL TEST CODE***********
    if (player2.getAButton())
      flywheel.run(true, true);
    else
      flywheel.run(false, false);
    */

    /*
    if (player2.getBButton()){
      turret.test("right");
    } else if(player2.getXButton()) {
      turret.test("left");
    }
    else{
      turret.test("stop");
    }

     if (player2.getXButton()){
       turret.test("left");
     }
     */
      
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    mechMaster.onDisable();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
