// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;

  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_leftRearMotor;
  private CANSparkMax m_rightRearMotor;
  
  private SparkMaxPIDController m_leftFrontController;
  private SparkMaxPIDController m_rightFrontController;
  private SparkMaxPIDController m_leftRearController;
  private SparkMaxPIDController m_rightRearController;

  private RelativeEncoder m_leftFrontEncoder;
  private RelativeEncoder m_rightFrontEncoder;
  private RelativeEncoder m_leftRearEncoder;
  private RelativeEncoder m_rightRearEncoder;


  private CANSparkMax t_leftFrontMotor;
  private CANSparkMax t_rightFrontMotor;
  private CANSparkMax t_leftRearMotor;
  private CANSparkMax t_rightRearMotor;

  private SparkMaxPIDController t_leftFrontController;
  private SparkMaxPIDController t_rightFrontController;
  private SparkMaxPIDController t_leftRearController;
  private SparkMaxPIDController t_rightRearController;

  private RelativeEncoder t_leftFrontEncoder;
  private RelativeEncoder t_rightFrontEncoder;
  private RelativeEncoder t_leftRearEncoder;
  private RelativeEncoder t_rightRearEncoder;
  
  private WPI_CANCoder left_front_CANCoder ;
  private WPI_CANCoder right_front_CANCoder ;
  private WPI_CANCoder left_rear_CANCoder ;
  private WPI_CANCoder right_rear_CANCoder ;

  private double kP;
  private double kI;
  private double kD;

  private CANSparkMax intake_arm_Motor;
  private CANSparkMax intake_right_Motor;
  private CANSparkMax intake_left_Motor;
  private SparkMaxPIDController intake_arm_Controller;
  private RelativeEncoder intake_arm_Encoder;

  private CANSparkMax Elevator_Motor;

  private SparkMaxPIDController elev_Controller;

  private RelativeEncoder elev_Encoder;

  private AHRS ahrs;





  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    kP = Constants.kP;
    kI = Constants.kI;
    kD = Constants.kD;
    m_stick = new Joystick(0);

    m_leftFrontMotor = new CANSparkMax(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
    m_leftFrontMotor.setInverted(true);
    m_rightFrontMotor = new CANSparkMax(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
    m_rightFrontMotor.setInverted(false);
    m_leftRearMotor = new CANSparkMax(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
    m_leftRearMotor.setInverted(false);
    m_rightRearMotor = new CANSparkMax(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);

    t_leftFrontMotor = new CANSparkMax(Constants.FRONT_LEFT_MODULE_STEER_MOTOR, MotorType.kBrushless);
    t_leftFrontMotor.setInverted(true);
    t_rightFrontMotor = new CANSparkMax(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, MotorType.kBrushless);
    t_rightFrontMotor.setInverted(true);
    t_leftRearMotor = new CANSparkMax(Constants.BACK_LEFT_MODULE_STEER_MOTOR, MotorType.kBrushless);
    t_leftRearMotor.setInverted(true);
    t_rightRearMotor = new CANSparkMax(Constants.BACK_RIGHT_MODULE_STEER_MOTOR, MotorType.kBrushless);
    t_rightRearMotor.setInverted(true);

    left_front_CANCoder = new WPI_CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
    right_front_CANCoder = new WPI_CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);
    left_rear_CANCoder = new WPI_CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
    right_rear_CANCoder = new WPI_CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);

    // intake

    intake_arm_Motor = new CANSparkMax(Constants.INTAKE.INTAKE_JOINT_MOTOR, MotorType.kBrushless);
    intake_arm_Motor.setIdleMode(IdleMode.kBrake);
    intake_arm_Motor.setInverted(true);
    intake_arm_Controller = intake_arm_Motor.getPIDController();
    intake_arm_Encoder = intake_arm_Motor.getEncoder();
    intake_arm_Controller.setSmartMotionMaxAccel(1500, 0);

    intake_right_Motor = new CANSparkMax(Constants.INTAKE.INTAKE_RIGHT_MOTOR, MotorType.kBrushed);
    intake_right_Motor.setIdleMode(IdleMode.kCoast);
    intake_left_Motor = new CANSparkMax(Constants.INTAKE.INTAKE_LEFT_MOTOR, MotorType.kBrushed);
    intake_left_Motor.setIdleMode(IdleMode.kCoast);
    intake_left_Motor.setInverted(true);

    //Elavator

    Elevator_Motor = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
    Elevator_Motor.setIdleMode(IdleMode.kBrake);
    elev_Encoder = Elevator_Motor.getEncoder();
    elev_Controller = Elevator_Motor.getPIDController();


   



    try {
      ahrs = new AHRS(SPI.Port.kMXP); 
    } 
    catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    ahrs.reset();

    intake_arm_Controller.setP(Constants.INTAKE.kP);
    intake_arm_Controller.setI(Constants.INTAKE.kI);
    intake_arm_Controller.setD(Constants.INTAKE.kD);
    intake_arm_Controller.setIZone(Constants.INTAKE.kIz);
    intake_arm_Controller.setFF(Constants.INTAKE.kFF);
    intake_arm_Controller.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    intake_arm_Encoder.setPosition(0);


    m_leftFrontController = m_leftFrontMotor.getPIDController();
    m_leftFrontEncoder = m_leftFrontMotor.getEncoder();
    m_leftFrontEncoder.setVelocityConversionFactor(1/Constants.driveRatio);
    m_leftFrontController.setP(Constants.Drive.kP);
    m_leftFrontController.setI(Constants.Drive.kI);
    m_leftFrontController.setD(Constants.Drive.kD);
    m_leftFrontController.setIZone(Constants.Drive.kIz);
    m_leftFrontController.setFF(Constants.Drive.kFF);
    m_leftFrontController.setOutputRange(-1, 1);

    // Right Front
    m_rightFrontController = m_rightFrontMotor.getPIDController();
    m_rightFrontEncoder = m_rightFrontMotor.getEncoder();
    
    
    m_rightFrontController.setP(Constants.kP);
    m_rightFrontController.setI(Constants.kI);
    m_rightFrontController.setD(Constants.kD);
    m_rightFrontController.setIZone(Constants.kIz);
    m_rightFrontController.setFF(Constants.kFF);
    m_rightFrontController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    // Left Rear
    m_leftRearController = m_leftRearMotor.getPIDController();
    m_leftRearEncoder = m_leftRearMotor.getEncoder();
    
    m_leftRearController.setP(Constants.kP);
    m_leftRearController.setI(Constants.kI);
    m_leftRearController.setD(Constants.kD);
    m_leftRearController.setIZone(Constants.kIz);
    m_leftRearController.setFF(Constants.kFF);
    m_leftRearController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    //Right Rear
    m_rightRearController = m_rightRearMotor.getPIDController();
    m_rightRearEncoder = m_rightRearMotor.getEncoder();
    
    m_rightRearController.setP(Constants.kP);
    m_rightRearController.setI(Constants.kI);
    m_rightRearController.setD(Constants.kD);
    m_rightRearController.setIZone(Constants.kIz);
    m_rightRearController.setFF(Constants.kFF);
    m_rightRearController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

// Turning

    t_leftFrontController = t_leftFrontMotor.getPIDController();
    t_leftFrontEncoder = t_leftFrontMotor.getEncoder();
    t_leftFrontEncoder.setPositionConversionFactor(Constants.steerRatio);
    t_leftFrontEncoder.setPosition((left_front_CANCoder.getPosition() - Constants.FRONT_LEFT_MODULE_STEER_OFFSET));

    t_leftFrontController.setP(Constants.kP);
    t_leftFrontController.setI(Constants.kI);
    t_leftFrontController.setD(Constants.kD);
    t_leftFrontController.setIZone(Constants.kFF);
    t_leftFrontController.setFF(Constants.kFF);
    t_leftFrontController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);


    t_rightFrontController = t_rightFrontMotor.getPIDController();
    t_rightFrontEncoder = t_rightFrontMotor.getEncoder();
    t_rightFrontEncoder.setPositionConversionFactor(Constants.steerRatio);
    t_rightFrontEncoder.setPosition((right_front_CANCoder.getPosition() - Constants.FRONT_RIGHT_MODULE_STEER_OFFSET));

    
    t_rightFrontController.setP(Constants.kP);
    t_rightFrontController.setI(Constants.kI);
    t_rightFrontController.setD(Constants.kD);
    t_rightFrontController.setIZone(Constants.kIz);
    t_rightFrontController.setFF(Constants.kFF);
    t_rightFrontController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    // Left Rear
    t_leftRearController = t_leftRearMotor.getPIDController();
    t_leftRearEncoder = t_leftRearMotor.getEncoder();
    t_leftRearEncoder.setPositionConversionFactor(Constants.steerRatio);
    t_leftRearEncoder.setPosition((left_rear_CANCoder.getPosition() - Constants.BACK_LEFT_MODULE_STEER_OFFSET));

    t_leftRearController.setP(Constants.kP);
    t_leftRearController.setI(Constants.kI);
    t_leftRearController.setD(Constants.kD);
    t_leftRearController.setIZone(Constants.kIz);
    t_leftRearController.setFF(Constants.kFF);
    t_leftRearController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    //Right Rear
    t_rightRearController = t_rightRearMotor.getPIDController();
    t_rightRearEncoder = t_rightRearMotor.getEncoder();
    t_rightRearEncoder.setPositionConversionFactor(Constants.steerRatio);
    t_rightRearEncoder.setPosition((right_rear_CANCoder.getPosition() - Constants.BACK_RIGHT_MODULE_STEER_OFFSET));

    t_rightRearController.setP(Constants.kP);
    t_rightRearController.setI(Constants.kI);
    t_rightRearController.setD(Constants.kD);
    t_rightRearController.setIZone(Constants.kIz);
    t_rightRearController.setFF(Constants.kFF);
    t_rightRearController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);


    SmartDashboard.putNumber("P Gain", Constants.kP);
    SmartDashboard.putNumber("I Gain", Constants.kI);
    SmartDashboard.putNumber("D Gain", Constants.kD);
    SmartDashboard.putNumber("I Zone", Constants.kIz);
    SmartDashboard.putNumber("Feed Forward", Constants.kFF);
    SmartDashboard.putNumber("Max Output", Constants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", Constants.kMinOutput);

    SmartDashboard.putNumber("Left Front", left_front_CANCoder.getPosition() - Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    SmartDashboard.putNumber("Right Front", right_front_CANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Left Rear", left_rear_CANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Right Rear", right_rear_CANCoder.getAbsolutePosition());

   



  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("Arm Position", intake_arm_Encoder.getPosition());

    SmartDashboard.putNumber("Angle", ahrs.getAngle());
    

  }

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

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    intake_arm_Encoder.setPosition(0);
    intake_arm_Controller.setReference(0, ControlType.kPosition);
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double x = -m_stick.getRawAxis(0);
    double y = -m_stick.getRawAxis(1);
    double turn = -m_stick.getRawAxis(4);

    boolean up = m_stick.getRawButton(1);
    boolean down = m_stick.getRawButton(2);

    boolean rest = m_stick.getRawButton(7);
    boolean middle =  m_stick.getRawButton(8);
    boolean high =  m_stick.getRawButton(9);



    boolean in = m_stick.getRawButton(6);
    boolean out = m_stick.getRawButton(4);
    boolean slow_in = m_stick.getRawButton(5);

    double elevator = m_stick.getRawAxis(7);
    if(rest){
      elev_Controller.setReference(Constants.elev_rest, ControlType.kPosition);
    }
    else if(middle){
      elev_Controller.setReference(Constants.elev_mid, ControlType.kPosition);

    }
    else if(high){
      elev_Controller.setReference(Constants.elev_high, ControlType.kPosition);
    }
    else if(Math.abs(elevator) > 0.1){
      Elevator_Motor.set(elevator);
    }


    if(up){
      intake_arm_Controller.setReference(Constants.INTAKE.IntakeUp, ControlType.kPosition);
    }
    else if (down){
      intake_arm_Controller.setReference(Constants.INTAKE.IntakeDown, ControlType.kPosition);
    }
    
    if(in){
      intake_right_Motor.set(1);
      intake_left_Motor.set(1);
    }
    else if (out){
      intake_right_Motor.set(-1);
      intake_left_Motor.set(-1);
    }

    else if (slow_in){
      intake_right_Motor.set(0.25);
      intake_left_Motor.set(0.25)
      ;
    }
    else{
      intake_right_Motor.set(0);
      intake_left_Motor.set(0);
    }

    if( Math.abs(x) < 0.1){
      x = 0;
    }

    if( Math.abs(y) < 0.1){
      y = 0;
    }

    if( Math.abs(turn) < 0.1){
      turn = 0;
    }

    SmartDashboard.putBoolean("Up", up);
    SmartDashboard.putBoolean("Down", down);

    SmartDashboard.putBoolean("In", in);
    SmartDashboard.putBoolean("Out", out);

    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("Turn", turn);
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Constants.MAX_VELOCITY_METERS_PER_SECOND * y , Constants.MAX_VELOCITY_METERS_PER_SECOND * x, Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * turn, ahrs.getRotation2d());

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(Units.degreesToRadians(t_leftFrontEncoder.getPosition())));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight, new Rotation2d(Units.degreesToRadians(t_rightFrontEncoder.getPosition())));
    var rearLeftOptimized = SwerveModuleState.optimize(backLeft, new Rotation2d(Units.degreesToRadians(t_leftRearEncoder.getPosition())));
    var rearRightOptimized = SwerveModuleState.optimize(backRight, new Rotation2d(Units.degreesToRadians(t_rightRearEncoder.getPosition())));
    
    t_leftFrontController.setReference(frontLeftOptimized.angle.getDegrees(), ControlType.kPosition);
    t_rightFrontController.setReference(frontRightOptimized.angle.getDegrees(), ControlType.kPosition);
    t_leftRearController.setReference(rearLeftOptimized.angle.getDegrees(), ControlType.kPosition);
    t_rightRearController.setReference(rearRightOptimized.angle.getDegrees(), ControlType.kPosition);
    //t_leftFrontController.setReference(frontLeftOptimized.angle.getDegrees(), ControlType.kPosition);
    //t_rightFrontController.setReference(frontRightOptimized.angle.getDegrees()*Constants.steerRatio, ControlType.kPosition);
    //t_leftRearController.setReference(rearLeftOptimized.angle.getDegrees()*Constants.steerRatio, ControlType.kPosition);
    //t_rightRearController.setReference(rearRightOptimized.angle.getDegrees()*Constants.steerRatio, ControlType.kPosition);

    m_leftFrontController.setReference(frontLeftOptimized.speedMetersPerSecond , ControlType.kVelocity);
    //m_leftFrontMotor.set(y);
    SmartDashboard.putNumber("FL Speed", m_leftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("FL SpeedSetPoint", frontLeftOptimized.speedMetersPerSecond);


    //m_rightFrontMotor.set(Constants.gain * frontRightOptimized.speedMetersPerSecond/Constants.MAX_VELOCITY_METERS_PER_SECOND);
    //m_leftRearMotor.set(Constants.gain* rearLeftOptimized.speedMetersPerSecond/Constants.MAX_VELOCITY_METERS_PER_SECOND);
    //.m_rightRearMotor.set(Constants.gain* rearRightOptimized.speedMetersPerSecond/Constants.MAX_VELOCITY_METERS_PER_SECOND);
    //m_rightFrontController.setReference(frontRightOptimized.speedMetersPerSecond*Constants.driveRatio, ControlType.kVelocity);
    //m_leftRearController.setReference(rearLeftOptimized.speedMetersPerSecond*Constants.driveRatio, ControlType.kVelocity);
    //m_rightRearController.setReference(rearRightOptimized.speedMetersPerSecond*Constants.driveRatio, ControlType.kVelocity);


    SmartDashboard.putNumber("I Zone", Constants.kIz);
    SmartDashboard.putNumber("Feed Forward", Constants.kFF);
    SmartDashboard.putNumber("Max Output", Constants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", Constants.kMinOutput);
    SmartDashboard.putNumber("Left Front", left_front_CANCoder.getPosition() - Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    SmartDashboard.putNumber("NEO Left Front", t_leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("SetPOint Left Front", frontLeftOptimized.angle.getDegrees());

    SmartDashboard.putNumber("NEO Left Front", t_leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("Left Front", left_front_CANCoder.getPosition() - Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    SmartDashboard.putNumber("Right Front", right_front_CANCoder.getPosition() - Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    SmartDashboard.putNumber("Left Rear", left_rear_CANCoder.getPosition()- Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    SmartDashboard.putNumber("Right Rear", right_rear_CANCoder.getPosition()- Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

    SmartDashboard.putNumber("NEO Right Front", t_rightFrontEncoder.getPosition());
    SmartDashboard.putNumber("NEO Left Rear", t_leftRearEncoder.getPosition());
    SmartDashboard.putNumber("NEO Right Rear", t_rightRearEncoder.getPosition());

    SmartDashboard.putNumber("Elevator Rest", Constants.elev_rest);
    SmartDashboard.putNumber("Elevator Middle", Constants.elev_mid);
    SmartDashboard.putNumber("Elevator High", Constants.elev_high); 
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    
    SmartDashboard.putNumber("NEO Left Front", t_leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("Left Front", left_front_CANCoder.getPosition() );
    SmartDashboard.putNumber("Right Front", right_front_CANCoder.getPosition() );
    SmartDashboard.putNumber("Left Rear", left_rear_CANCoder.getPosition());
    SmartDashboard.putNumber("Right Rear", right_rear_CANCoder.getPosition());

    SmartDashboard.putNumber("NEO Right Front", t_rightFrontEncoder.getPosition());
    SmartDashboard.putNumber("NEO Left Rear", t_leftRearEncoder.getPosition());
    SmartDashboard.putNumber("NEO Right Rear", t_rightRearEncoder.getPosition());

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
