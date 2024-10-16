package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.OI;
import frc.robot.oi.OI.Buttons;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.constants.GameConstants.GamePiece;

import java.util.function.DoubleSupplier;

public final class Controls {

    public static final class DriverControls {
        public static final DoubleSupplier SwerveForwardAxis = () -> {
            OI oi = OI.getInstance();
            return oi.driverController().getAxis(OI.Axes.LEFT_STICK_Y) * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveStrafeAxis = () -> {
            OI oi = OI.getInstance();
            return -oi.driverController().getAxis(OI.Axes.LEFT_STICK_X) * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveRotationAxis = () -> {
            OI oi = OI.getInstance();
            return -oi.driverController().getAxis(OI.Axes.RIGHT_STICK_X);
        };
        public static final Trigger leftSubstation = new Trigger(() -> {
            OI oi = OI.getInstance();
            return oi.driverController().getButton(Buttons.LEFT_TRIGGER).getAsBoolean();
        });
        public static final Trigger rightSubstation = new Trigger(() -> {
            OI oi = OI.getInstance();
            return oi.driverController().getButton(Buttons.RIGHT_TRIGGER).getAsBoolean();
        });

        public static final Trigger ResetGyroButton1 = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.A_BUTTON).getAsBoolean();
                }),
                ResetGyroButton2 = new Trigger(
                        () -> {
                            OI oi = OI.getInstance();
                            return oi.driverController().getButton(Buttons.B_BUTTON).getAsBoolean();
                        });
    }

    public static final class OperatorControls {
        private static GamePiece gamePiece = GamePiece.CUBE;
        public static GamePiece getQueuedGamePiece() {
            return gamePiece;
        }
        static {
            OI.getInstance().operatorController().getButton(Buttons.LEFT_TRIGGER).whileTrue(
                    Commands.runOnce(() -> {
                        gamePiece = GamePiece.CUBE;
                    })
            );
            OI.getInstance().operatorController().getButton(Buttons.RIGHT_TRIGGER).whileTrue(
                    Commands.runOnce(() -> {
                        gamePiece = GamePiece.CONE;
                    })
            );
        }
    }

    public static double rumbleStrength = 0.5;
}
