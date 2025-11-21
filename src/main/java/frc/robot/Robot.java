package frc.robot;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.commands.Auton;
import frc.robot.commands.Elevator;
import frc.robot.commands.Manipulator;
import frc.robot.constants.PIDVar;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.autonConst;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.SPI.Port;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANrange;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Robot extends TimedRobot {
  //motors
  public static final SparkMax manRight = new SparkMax(31, MotorType.kBrushless);
  public static final SparkMax manLeft = new SparkMax(32, MotorType.kBrushless);
  public static final SparkMax elevatorR = new SparkMax(21, MotorType.kBrushless);
  public static final SparkMax elevatorL = new SparkMax(22, MotorType.kBrushless);
  public static SparkMax right1 = new SparkMax(11, MotorType.kBrushless);
  public static SparkMax right2 = new SparkMax(12, MotorType.kBrushless);
  public static SparkMax left1 = new SparkMax(13, MotorType.kBrushless);
  public static SparkMax left2 = new SparkMax(14, MotorType.kBrushless);


  //PID encoders
  public static final RelativeEncoder drvLEnc = left1.getEncoder();
  public static final RelativeEncoder drvREnc = right1.getEncoder();

  //sensors
  public static final RelativeEncoder elevatorEnc = elevatorR.getEncoder();
  public static final CANrange backCanRange = new CANrange(0);
  public static final CANrange sideCanRange = new CANrange(1);

  //Feedforward
  public static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);




  //PID
  public static final SparkClosedLoopController elevatorRPID=manRight.getClosedLoopController(); //left follows


  public static final DigitalInput stg2Top = new DigitalInput(0);
  public static final DigitalInput CarrigeTop = new DigitalInput(1);
  public static final DigitalInput CarrigeBottom = new DigitalInput(2);
  public final BooleanSupplier CarriageBottom2 = () -> CarrigeBottom.get();

  // gyroscope
  public static final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // camera
  public static final UsbCamera camera = CameraServer.startAutomaticCapture();
  // controllers
  public static final PS5Controller DRIV_CONTROLLER = new PS5Controller(0);
  public static final XboxController OPPERA_CONTROLLER = new XboxController(1);

  // Timers :(
  public static final Timer drivModeTimer=new Timer();
  public static final Timer autonTimer = new Timer();

  //Mode
  public String mode = "";

  //Slew
  SlewRateLimiter filter = new SlewRateLimiter(0.5);


  //Shuffleboard
  // private ShuffleboardTab tab = Shuffleboard.getTab("Main");
  // public GenericEntry elevatorSetP = tab.addPersistent("ElevatorP",.001).getEntry();
  // public GenericEntry elevatorSetI = tab.addPersistent("ElevatorI",0).getEntry();
  // public GenericEntry elevatorSetD = tab.addPersistent("ElevatorD",.001).getEntry();
  // public SimpleWidget elevatorPosition = tab.add("Elevator Position", elevatorR.getEncoder());
  // public SimpleWidget roll = tab.add("Roll Angle", gyro.getRoll());
  // public SimpleWidget pitch = tab.add("Pitch Angle", gyro.getPitch());
  // public SimpleWidget yaw = tab.add("Yaw Angle", gyro.getYaw());

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    //CANrange

    //SmartDashboard
    SmartDashboard.updateValues();
    SmartDashboard.setDefaultNumber("Elevator Encoder (rot)", elevatorEnc.getPosition());
    SmartDashboard.setDefaultNumber("Navx Roll", gyro.getRoll());
    SmartDashboard.setDefaultNumber("Navx Yaw", gyro.getYaw());
    SmartDashboard.setDefaultNumber("Navx Pitch", gyro.getPitch());
    SmartDashboard.setDefaultNumber("RPS of Left Motor", drvLEnc.getVelocity()*60);
    SmartDashboard.setDefaultNumber("RPS of Right Motor", drvREnc.getVelocity()*60);


    //SparkMaxConfig
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    SparkMaxConfig configR2 = new SparkMaxConfig();
    configR2.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(right1);
    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true);
    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true).follow(left1);
    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);

    //ELEVATOR
    SparkMaxConfig configEleR = new SparkMaxConfig();
    configEleR.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false).closedLoop
    .pid(PIDVar.elevatorP,
        PIDVar.elevatorI,
        PIDVar.elevatorD,
        ClosedLoopSlot.kSlot0);
    SparkMaxConfig configEleL = new SparkMaxConfig();
    configEleL.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(elevatorR, true).closedLoop
    .pid(PIDVar.elevatorP,
        PIDVar.elevatorI,
        PIDVar.elevatorD,
        ClosedLoopSlot.kSlot0);
    elevatorR.configure(configEleR, null, null);
    elevatorL.configure(configEleL, null, null);

    //MANIPULATOR
    SparkMaxConfig configManRight = new SparkMaxConfig();
    configManRight.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true);

    SparkMaxConfig configManLeft = new SparkMaxConfig();
    configManLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    manRight.configure(configManRight, null, null);
    manLeft.configure(configManLeft, null, null);

  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (mode == "auton") {
      UpdatePeriodic.updateSensorValues();
    } else {
      UpdatePeriodic.updateSensorValues();
      new UpdatePeriodic().updateControllerInputs();
    }
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    mode = "auton";
    gyro.reset();
    // starting and reseting the timer used in auton
    autonTimer.reset();
    // pid (right)
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    // .closedLoop
    // .pid(PIDVar.drvRightP,
    //     PIDVar.drvRightI,
    //     PIDVar.drvRightD,
    //     ClosedLoopSlot.kSlot0);
    SparkMaxConfig configR2 = new SparkMaxConfig();

    configR2.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(false).follow(right1);
    // .closedLoop
    // .pid(PIDVar.autonLeftP,
    //     PIDVar.autonLeftI,
    //     PIDVar.autonLeftD,
    //     ClosedLoopSlot.kSlot0);

    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    // pid (left)
    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(true);
    // .closedLoop
    // .pid(PIDVar.autonLeftP,
    //     PIDVar.autonLeftI,
    //     PIDVar.autonLeftD,
    //     ClosedLoopSlot.kSlot0);

    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(true).follow(left1);
    // .closedLoop
    // .pid(PIDVar.autonLeftP,
    //     PIDVar.autonLeftI,
    //     PIDVar.autonLeftD,
    //     ClosedLoopSlot.kSlot0);

    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }
  
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // UpdatePeriodic.updateControllerInputs();
    UpdatePeriodic.updateSensorValues();
    UpdatePeriodic.updateShuffleboardValues();
    new UpdatePeriodic().ABXYDpadUpdate();
    new UpdatePeriodic().updateControllerInputs();
    // newTabKevin.add("Elevator Height ", RobotConstants.elevatorHeight);

    // ELEVATOR
    if (RobotConstants.bottEndstop == true) {
      elevatorEnc.setPosition(0);
      // System.out.println("At bottom, reseting ENC zero");
    }    
    // } else 
    if (OPPERA_CONTROLLER.getPOV() == 0 ) { // DPAD movment (manual)

      if (RobotConstants.elevatorRotHeight > RobotConstants.maxHeightforSlowThreshold) { //at top
        RobotConstants.elevatorOutput = (0.1);
    } else if (RobotConstants.elevatorRotHeight < RobotConstants.minHeightforDecelerationThreshold) { //at bottom
        RobotConstants.elevatorOutput = (0.2);
    } else {
        RobotConstants.elevatorOutput = (.6);
      }
    

    } else if (OPPERA_CONTROLLER.getPOV() == 180 || OPPERA_CONTROLLER.getLeftBumperButton()) {

      if (RobotConstants.elevatorRotHeight < .1) 
        {RobotConstants.elevatorOutput = -0.01;}
      else if (RobotConstants.elevatorRotHeight < RobotConstants.minHeightforDecelerationThreshold) 
        {RobotConstants.elevatorOutput = -0.1;}
      else{
      RobotConstants.elevatorOutput = -0.5;
      }

    } else {
      RobotConstants.elevatorOutput = 0.03;
    }

    // // digital stops
    // if ((RobotConstants.OpperaDPadUp || RobotConstants.OpperaDPadUpRight) && RobotConstants.topEndstop == true) {
    //   RobotConstants.elevatorOutput = 0;
    //   System.err.println("ERROR: TRYING TO OVER EXTEND ELEVATOR, setting elevator speed to 0");


    // } else if ((RobotConstants.OpperaDPadDown)
    //     && RobotConstants.bottEndstop == true) {
    //   RobotConstants.elevatorOutput = 0;
    //   System.err.println("ERROR: TRYING TO UNDER EXTEND ELEVATOR, setting elevator speed to 0");

    // }


    elevatorR.set(RobotConstants.elevatorOutput); // one of the only times the elevator speed actually gets set in the code


    // MANIPULATOR   
    if (OPPERA_CONTROLLER.getRightTriggerAxis() > 0) { // intake
      // manLeftPID.setReference(-RobotConstants.manLeftOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      // manRightPID.setReference(-RobotConstants.manRightOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      manLeft.set(OPPERA_CONTROLLER.getRightTriggerAxis() *RobotConstants.manMaxSPD);
      manRight.set(OPPERA_CONTROLLER.getRightTriggerAxis()*RobotConstants.manMaxSPD);

    } else if (OPPERA_CONTROLLER.getLeftTriggerAxis() > 0) {
      // manLeftPID.setReference(RobotConstants.manLeftOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      // manRightPID.setReference(RobotConstants.manRightOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      manLeft.set(-OPPERA_CONTROLLER.getLeftTriggerAxis()*RobotConstants.manMaxSPD);
      manRight.set(-OPPERA_CONTROLLER.getLeftTriggerAxis()*RobotConstants.manMaxSPD);

    } else if (RobotConstants.OpperabButton) {
      // manLeftPID.setReference(-(RobotConstants.manLeftOutput/2)*RobotConstants.manMaxSPD,ControlType.kVelocity);
      // manRightPID.setReference(-(RobotConstants.manRightOutput/2)*RobotConstants.manMaxSPD,ControlType.kVelocity);
      manLeft.set(.7);
      manRight.set(.7);

    } else {// manual control
      manLeft.set(Math.abs(OPPERA_CONTROLLER.getLeftY())*OPPERA_CONTROLLER.getLeftY());
      manRight.set(Math.abs(OPPERA_CONTROLLER.getRightY())*OPPERA_CONTROLLER.getRightY());
      RobotConstants.spazMode = true;
    }
  
    // DRIVE

    RobotConstants.slowMode ^= DRIV_CONTROLLER.getL1Button();
    RobotConstants.turboMode ^= DRIV_CONTROLLER.getL2Button();


    if (RobotConstants.slowMode && RobotConstants.turboMode) { // if both pressed, make it neither
      RobotConstants.turboMode = false;
      RobotConstants.slowMode = false;
    }

    if (RobotConstants.slowMode) { // slowmode max speed
      RobotConstants.robotAccMaxSpeed = RobotConstants.slowModeMaxSpeed;
    } else if (RobotConstants.turboMode) {
      RobotConstants.robotAccMaxSpeed = 1;
    } else {
      RobotConstants.robotAccMaxSpeed = RobotConstants.robotMaxSpeed;
    }


    if (DRIV_CONTROLLER.getLeftY() > 0 && DRIV_CONTROLLER.getRightY() > 0) { // if backwards make slow mode slower
      RobotConstants.turboMode = false;
      RobotConstants.slowModeMaxSpeed = 0.1;
    } else {
      RobotConstants.slowModeMaxSpeed = 0.125;
    }

  

    if (DRIV_CONTROLLER.getL2Axis() > .05) {
      RobotConstants.leftOutput = DRIV_CONTROLLER.getL2Axis()*DRIV_CONTROLLER.getL2Axis()+.01;
      RobotConstants.rightOutput = DRIV_CONTROLLER.getL2Axis()*DRIV_CONTROLLER.getL2Axis()+.01;
      //System.out.println("going from left trigger");

    } else if (DRIV_CONTROLLER.getR2Axis() > .05) {
      RobotConstants.leftOutput = -DRIV_CONTROLLER.getR2Axis()*DRIV_CONTROLLER.getR2Axis()+.01;
      RobotConstants.rightOutput = -DRIV_CONTROLLER.getR2Axis()*DRIV_CONTROLLER.getR2Axis()+.01;
      //System.out.println("going from right trigger");

    } else {
      RobotConstants.leftOutput = Math.abs(DRIV_CONTROLLER.getLeftY()) * DRIV_CONTROLLER.getLeftY();
      RobotConstants.rightOutput = Math.abs(DRIV_CONTROLLER.getRightY()) * DRIV_CONTROLLER.getRightY();

      //System.out.println("going from normal controls");
    }


    //ANTI-TIP
    // if (gyro.getPitch() > 25) {
    //   left1.set(.75);
    //   right1.set(.75);
    //   System.err.println("ANTI-TIP ACTIVATED");
    // } else if (gyro.getPitch() < -25) {
    //   left1.set(.5);
    //   right1.set(.5);
    //   System.err.println("ANTI-TIP ACTIVATED");

    // }




    left1.set(RobotConstants.leftOutput * RobotConstants.robotAccMaxSpeed);
    right1.set(RobotConstants.rightOutput * RobotConstants.robotAccMaxSpeed);


  }
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    System.out.println("Code is running");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {


  }


}
