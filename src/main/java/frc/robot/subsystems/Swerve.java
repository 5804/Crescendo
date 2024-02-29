package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
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
                () -> {return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());}, // ChassisSpeeds supplier
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(2.45, 0.0, 0.9), // Translation PID constants -- P = 2.45
                        new PIDConstants(1.225, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
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

    public void driveRobotRelative(ChassisSpeeds autoChassisSpeeds) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(autoChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
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

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
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
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}

/*     public class Limelight extends SubsystemBase {
        double tx = 0;
        double ty = 0;
        double tv = 0;
        double ta = 0;
        private SendableChooser<Boolean> m_limelightSwitch = new SendableChooser<>();

        public Limelight() {
            m_limelightSwitch.setDefaultOption("On", true);
            m_limelightSwitch.addOption("Off", false);

            SmartDashboard.putData("Limelight Switch", m_limelightSwitch);
        }

        //rotation for vision tracking-- we need to correct rotation on the limelight

        @Override
        public void periodic() {
            tx =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("tx")
                .getDouble(0);
            SmartDashboard.putNumber("tx", tx);

            ty =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("ty")
                .getDouble(0);
            SmartDashboard.putNumber("ty", ty);

            tv =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("tv")
                .getDouble(0);
            SmartDashboard.putBoolean("tv", tv >= 1.0);

            ta =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("ta")
                .getDouble(0);
            SmartDashboard.putBoolean("target valid", ta >= 1.0);

            SmartDashboard.putNumber("getSteeringValue", getSteeringValue());
        }

        public void setToAprilTags() {
            NetworkTableInstance
            .getDefault()
            .getTable("limelight")
            .getEntry("pipeline")
            .setNumber(0.0);
        }

        public void setToRetroreflectiveTape() {
            NetworkTableInstance
            .getDefault()
            .getTable("limelight")
            .getEntry("pipeline")
            .setNumber(1.0);
        }

        public double getSteeringValue() {
            double STEER_K = 0.1;

            double signumtx = Math.signum(tx);

            // if tv = 0, target is not valid so return 0.0
            if (tv == 0) {
            return 0.0;
            }

            if (m_limelightSwitch.getSelected() == false) {
            return 0.0;
            }

            double txAbs = Math.abs(tx);
            double txDeadband = txAbs - Constants.LIMELIGHT_DEADBAND;

            if (txDeadband < 0) {
            return 0.0;
            }

            double minDriveWithSine = signumtx * Constants.MIN_STEER_K;
            double steer_cmd = tx * STEER_K;
            double finalSteerCmd = minDriveWithSine + steer_cmd;

            return finalSteerCmd;
        }
    }
}

*/