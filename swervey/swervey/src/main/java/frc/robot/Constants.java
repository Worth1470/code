package frc.robot;

public class Constants {
    public static final double gain = 0.6;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.54;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.72;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.4196;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public final static double kMaxOutput = 1;
    public final static double kMinOutput = -1;
    public final double maxRPM = 5700;

    public static final double driveRatio = 6.75*200*0.95;
    public static final double steerRatio = 16.8;
    public static class Drive{
      public static double kP = 1.5;
      public static double kI = 0.000000;
      public static double kD = 0.00000;
       public static double kIz = 0;
      public static double kFF = 0.0000;
    }
    public static double kP = 0.0070;
    public static double kI = 0.000000;
    public static double kD = 0.00000;
    public static double kIz = 0;
    public static double kFF = 0.0001;

    public double kPLF , kILF, kDLF, kIzLF, kFFLF;
    public double kPRF, kIRF, kDRF, kIzRF, kFFRF;
    public double kPLR, kILR, kDLR, kIzLR, kFFLR;
    public double kPRR, kIRR, kDRR, kIzRR, kFFRR;

    public double t_kP, t_kI, t_kD, t_kIz, t_kFF;

    public int joystickPort = 0;
    public static class INTAKE{

      public static final int INTAKE_JOINT_MOTOR = 1; // ?
      public static final int INTAKE_RIGHT_MOTOR = 5;
      public static final int INTAKE_LEFT_MOTOR = 30;
      
      public static double kP = 0.070;
      public static double kI = 0.000000;
      public static double kD = 0.00000;
      public static double kIz = 0;
      public static double kFF = 0.0002;
      public static double IntakeUp = 0; // ??
      public static double IntakeDown = 17; // ??
      
    }
  //Left Front
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 29;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 28;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 101.162;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 24;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 102.39;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 35;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 25;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 36.123;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 26;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 27;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 118.56;


    //ELEVATOR

    public static final int ELEVATOR_MOTOR = 31; // ????
    public static final double elev_rest = 0; // ???
    public static final double elev_mid = 0; // ???
    public static final double elev_high = 0; // ??? 


}
