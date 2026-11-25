package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "REVDuo_Auto_Alternate2026 (Blocks to Java)", preselectTeleOp = "REVDuo_Doubles2026")
public class RevDuoAutoAlternate extends LinearOpMode {

    private DcMotor rightDrive;
    private DcMotor flywheel;
    private Servo pollenServo;
    private DcMotor leftDrive;
    private VoltageSensor ControlHub_VoltageSensor;
    private DcMotor intakeMotor;
    private Servo flyGate;

    double targetDriveSpeed;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        int targetVelocity;
        ElapsedTime autoTimer;
        double kF;

        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        pollenServo = hardwareMap.get(Servo.class, "pollenServo");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flyGate = hardwareMap.get(Servo.class, "flyGate");

        // Put initialization blocks here.
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        targetVelocity = 1160;
        autoTimer = new ElapsedTime();
        waitForStart();
        if (opModeIsActive()) {
            // Ensures the servo for the pollen poker is in the right position pointing up
            pollenServo.setPosition(1);
            // Enable this block (right click) to add a delay to the program starting
            // The default delay is 10 seconds so be sure to adjust based on your strategy!
            // Robot drives forward 700 milliseconds to no longer touch the wall
            autoTimer.reset();
            while (autoTimer.milliseconds() <= 700 && opModeIsActive()) {
                leftDrive.setPower(-0.25);
                rightDrive.setPower(-0.25);
            }
            // Turn off drivetrain motors to prevent extra movement
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(10000);
            // Flywheel begins launch sequence for the preloaded pollen
            autoTimer.reset();
            // The flywheel is set to spin for 15 seconds.
            while (autoTimer.seconds() <= 8 && opModeIsActive()) {
                ((DcMotorEx) flywheel).setVelocity(targetVelocity);
                kF = 19.2 * (12 / ControlHub_VoltageSensor.getVoltage());
                ((DcMotorEx) flywheel).setVelocityPIDFCoefficients(80, 0, 0, kF);
                // The flywheel must be within a specified range of the target velocity for the intake and feeder servo to activate!
                if (((DcMotorEx) flywheel).getVelocity() >= targetVelocity - 10 && ((DcMotorEx) flywheel).getVelocity() <= targetVelocity + 50) {
                    intakeMotor.setPower(1);
                    flyGate.setPosition(0.8);
                } else {
                    // If the velocity is not within range, the intake and feeder servo turn off to prevent early feeding
                    flyGate.setPosition(0);
                    intakeMotor.setPower(0);
                }
            }
            ((DcMotorEx) flywheel).setVelocity(0);
            intakeMotor.setPower(0);
            flyGate.setPosition(0);
            goBackward(500);
            turnLeftBackward(450);
            goBackward(2900);
        }
    }

    /**
     * Describe this function...
     */
    private void goBackward(int time2) {
        targetDriveSpeed = 1 * (12.5 / ControlHub_VoltageSensor.getVoltage());
        leftDrive.setPower(-0.3);
        rightDrive.setPower(-0.32);
        sleep(time2);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
    }

    /**
     * Describe this function...
     */
    private void turnRightBackward(long time2) {
        targetDriveSpeed = 1 * (12.5 / ControlHub_VoltageSensor.getVoltage());
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(time2);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
    }

    /**
     * Describe this function...
     */
    private void turnLeftBackward(int time2) {
        targetDriveSpeed = 1 * (12.5 / ControlHub_VoltageSensor.getVoltage());
        leftDrive.setPower(0.5);
        rightDrive.setPower(-0.5);
        sleep(time2);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
    }
}
