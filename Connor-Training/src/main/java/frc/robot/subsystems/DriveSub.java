package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSub extends SubsystemBase{
    private static final int kLeftDriveMotorID = 1;
    private static final int kRightDriveMotorID = 1;
    private static final int kPigeonID = 0;

    private SparkMax leftDriveMotor = new SparkMax(kLeftDriveMotorID, MotorType.kBrushless);
    private SparkMax rightDriveMotor = new SparkMax(kRightDriveMotorID, MotorType.kBrushless);
    private Pigeon2 gyro = new Pigeon2(kPigeonID);

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0));
    DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), Units.inchesToMeters(getLeftEncoderDistance()), Units.inchesToMeters(getRightEncoderDistance()), new Pose2d());

    private double getLeftEncoderDistance() {
        return leftDriveMotor.getEncoder().getPosition() * 4 * Math.PI;
    }

    private double getRightEncoderDistance() {
        return rightDriveMotor.getEncoder().getPosition() * 4 * Math.PI;
    }

    public DriveSub() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35);
        config.idleMode(IdleMode.kBrake);
        leftDriveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightDriveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Pose2d getPose() {
        return differentialDrivePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        differentialDrivePoseEstimator.resetPose(pose);
    }

    @Override
    public void periodic() {
        differentialDrivePoseEstimator.update(gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    }
}
