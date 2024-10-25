package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // Creates a SysIdRoutine

    public void voltageDrive(Measure<Voltage> volts) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setSpeed(volts);
            mod.mAngleMotor.setControl(mod.anglePosition.withPosition(0));
        }
    }

    public void logMotors() {
        
    }

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));

    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));

    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    public SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(
        null,
        null,
        null,

        (state)->SignalLogger.writeString("state", state.toString())
    ),
    new SysIdRoutine.Mechanism(this::voltageDrive,
    null
    // log -> {        
    //     SwerveModule mod = mSwerveMods[0];
    //     double voltage = mod.mDriveMotor.getMotorVoltage().getValueAsDouble();
    //     log.motor("frontLeft")
    //         .voltage(m_appliedVoltage.mut_replace(voltage * RobotController.getBatteryVoltage(), Volts))
    //         .linearPosition(m_distance.mut_replace(mod.mDriveMotor.getPosition().getValueAsDouble(), Meters))
    //         .linearVelocity(m_velocity.mut_replace(mod.mDriveMotor.getVelocity().getValueAsDouble(), MetersPerSecond));

    //     mod = mSwerveMods[1];
    //     voltage = mod.mDriveMotor.getMotorVoltage().getValueAsDouble();
    //     log.motor("frontRight")
    //         .voltage(m_appliedVoltage.mut_replace(voltage * RobotController.getBatteryVoltage(), Volts))
    //         .linearPosition(m_distance.mut_replace(mod.mDriveMotor.getPosition().getValueAsDouble(), Meters))
    //         .linearVelocity(m_velocity.mut_replace(mod.mDriveMotor.getVelocity().getValueAsDouble(), MetersPerSecond));
        
    //     mod = mSwerveMods[2];
    //     voltage = mod.mDriveMotor.getMotorVoltage().getValueAsDouble();
    //     log.motor("backLeft")
    //         .voltage(m_appliedVoltage.mut_replace(voltage * RobotController.getBatteryVoltage(), Volts))
    //         .linearPosition(m_distance.mut_replace(mod.mDriveMotor.getPosition().getValueAsDouble(), Meters))
    //         .linearVelocity(m_velocity.mut_replace(mod.mDriveMotor.getVelocity().getValueAsDouble(), MetersPerSecond));
        
    //     mod = mSwerveMods[3];
    //     voltage = mod.mDriveMotor.getMotorVoltage().getValueAsDouble();
    //     log.motor("backRight")
    //         .voltage(m_appliedVoltage.mut_replace(voltage * RobotController.getBatteryVoltage(), Volts))
    //         .linearPosition(m_distance.mut_replace(mod.mDriveMotor.getPosition().getValueAsDouble(), Meters))
    //         .linearVelocity(m_velocity.mut_replace(mod.mDriveMotor.getVelocity().getValueAsDouble(), MetersPerSecond));

    // }
    , this)

);


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Swerve() {
        setName("FrontLeftDrive");

        gyro = new Pigeon2(Constants.Swerve.pigeonID, "torch");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), // front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), // front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), // back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants)  // back right
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeed, // ChassisSpeeds supplier
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        // new PIDConstants(20, 0.0, 0.5), // Translation PID constants -- P = 2.45
                        // new PIDConstants(15, 0.0, 0.15), // Rotation PID constants
                        new PIDConstants(1.5, 0, 0), // 6 // 2
                        new PIDConstants(1.5, 0, 0), // 6 // 3
                        4, // Max module speed, in m/s // 3
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module. //3
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            mSwerveMods[0].mDriveMotor.getPosition(),
            mSwerveMods[0].mDriveMotor.getVelocity(),
            mSwerveMods[0].mDriveMotor.getMotorVoltage());

            mSwerveMods[0].mDriveMotor.optimizeBusUtilization();

            SignalLogger.start();
            
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void driveLeft() {
        drive(new Translation2d(0, 0.3), 0, true, false);
    }

    public void driveRight() {
        drive(new Translation2d(0, -0.3), 0, true, false);
    }

    public void driveForward() {
        drive(new Translation2d(0.3, 0), 0, true, false);
    }

    public void driveBack() {
        drive(new Translation2d(-0.3, 0), 0, true, false);
    }
    // public void driveRobotRelative(ChassisSpeeds autoChassisSpeeds) {
    //     SwerveModuleState[] swerveModuleStates =
    //         Constants.Swerve.swerveKinematics.toSwerveModuleStates(autoChassisSpeeds);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    //     for(SwerveModule mod : mSwerveMods){
    //         mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    //     }
    // }

    
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
  
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
          states[i] = mSwerveMods[i].getState();
        }
        return states;
      }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
          positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
      }

    public void setPose(Pose2d pose) {
        //swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        swerveOdometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);

    }

    public ChassisSpeeds getSpeed() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    // Limelight
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-vision");
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 31; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 10.25; 

    // distance from the target to the floor
    public double goalHeightInches = 53.875; //58

    public double stageHeightInches = 48.8125; // UNUSED

    //calculate distance from speaker
    public double calculateDistanceFromSpeaker() { 
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    //calculate angle to speaker
    // public double calculateAngleToSpeaker() { 
    //     NetworkTableEntry ty = table.getEntry("ty");
    //     double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    //     double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     //double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //     return angleToGoalDegrees;
    // }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    public double limelight_aim_proportional()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.025; // 0.001 // 0.02 // 0.025

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = table.getEntry("tx").getDouble(0.0) * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= (Math.PI * 2); // 12 // 2

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    public double limelight_aim_pass()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .002;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = table.getEntry("tx").getDouble(0.0) + 15 * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= (Math.PI * 12);

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    public Command limeLightAutoAlign() {
        return run(
            () -> {
                double rotationValue = limelight_aim_proportional();
                drive(
                    new Translation2d(0, 0), 
                    rotationValue, 
                    false,
                    true
                );
            }
        );
        // .until(() -> {return table.getEntry("tx").getDouble(0.0) > -0.05 && table.getEntry("tx").getDouble(0.0) < 0.05;});
    }

    public Command limelightStageAlign() {
        return run(
            () -> {
                double rotationValue = limelight_aim_proportional() + 15;
                drive(
                    new Translation2d(0, 0), 
                    rotationValue, 
                    false,
                    true
                );
            }
        );
        // .until(() -> {return table.getEntry("tx").getDouble(0.0) > -0.05 && table.getEntry("tx").getDouble(0.0) < 0.05;});
    }

    @Override
    public void periodic(){
        //double distanceFromSpeaker = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Odometry X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle", swerveOdometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("ty", table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Distance from Speaker", calculateDistanceFromSpeaker());
    }

}


