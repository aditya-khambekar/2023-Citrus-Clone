package frc.lib.swerve.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleStatesSwerveRequest implements SwerveRequest {
    public SwerveModuleState[] states;
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;

    public ModuleStatesSwerveRequest() {
        this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        this.SteerRequestType = SteerRequestType.MotionMagic;
        states = new SwerveModuleState[]{
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for(int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], this.DriveRequestType, this.SteerRequestType);
        }

        return StatusCode.OK;
    }

    public ModuleStatesSwerveRequest withModuleStates(SwerveModuleState[] states) {
        this.states = states;
        return this;
    }

    public ModuleStatesSwerveRequest withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    public ModuleStatesSwerveRequest withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}