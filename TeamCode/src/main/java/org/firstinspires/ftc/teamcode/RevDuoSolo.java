package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "REVDuo_Solo")
public class RevDuoSolo extends LinearOpMode {

    private DcMotor flywheel;
    private Servo flyGate;
    private DcMotor intakeMotor;
    private DcMotor rightDrive;
    private Servo pollenServo;
    private VoltageSensor ControlHub_VoltageSensor;
    private DcMotor leftDrive;

    int targetVelocity;

    /**
     * This function contains the controls for the flywheel system.
     */
    private void flywheelControls() {
        if (gamepad1.triangle) {
            // Manual control for flywheel motor
            ((DcMotorEx) flywheel).setVelocity(targetVelocity);
        } else if (gamepad1.square) {
            // Reverse flywheel
            flywheel.setPower(-1);
        } else if (gamepad1.left_bumper) {
            ((DcMotorEx) flywheel).setVelocity(targetVelocity);
            // Auto feed into flywheel if within a set velocity range
            if (((DcMotorEx) flywheel).getVelocity() >= targetVelocity - 0 && ((DcMotorEx) flywheel).getVelocity() <= targetVelocity + 20) {
                flyGate.setPosition(0.8);
            }
        } else if (gamepad1.dpad_up) {
            // Manual control for feeder servo
            flyGate.setPosition(0.8);
        } else if (gamepad1.dpad_down) {
            // Reverse feeder
            flyGate.setPosition(0.8);
        } else {
            // Turn off servo and motor when gamepad is not active
            flywheel.setPower(0);
            flyGate.setPosition(0);
        }
    }

    /**
     * This function contains the controls for the intake system using the Core Hex.
     */
    private void intakeControls() {
        // Intake in and out
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1);
        } else if (gamepad1.right_trigger_pressed) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
     * Blocks show where to place Initialization code (runs once, after touching the DS INIT
     * button, and before touching the DS Start arrow), Run code (runs once, after touching
     * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
     */
    @Override
    public void runOpMode() {
        double kF;
        double batteryPercentage;

        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flyGate = hardwareMap.get(Servo.class, "flyGate");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        pollenServo = hardwareMap.get(Servo.class, "pollenServo");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");

        // Setting up motors and variables
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        targetVelocity = 1160;
        waitForStart();
        if (opModeIsActive()) {
            // Ensures the servo is up
            pollenServo.setPosition(1);
            // Loop for the OpMode containing the functions
            while (opModeIsActive()) {
                kF = 19.2 * (12 / ControlHub_VoltageSensor.getVoltage());
                ((DcMotorEx) flywheel).setVelocityPIDFCoefficients(80, 0, 0, kF);
                intakeControls();
                pollenPoker();
                arcadeDrive();
                flywheelControls();
                // Allows for velocity to be seen on the Driver Station
                batteryPercentage = ((ControlHub_VoltageSensor.getVoltage() - 11.5) / 2) * 100;
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Battery", Math.round(batteryPercentage));
                telemetry.update();
            }
        }
    }

    /**
     * This function contains the code for the servo powered pollen poker on the front of the robot.
     */
    private void pollenPoker() {
        // Servo control for poking out pollen
        if (gamepad1.crossWasPressed()) {
            pollenServo.setPosition(0.32);
        } else if (gamepad1.circleWasPressed()) {
            pollenServo.setPosition(1);
        }
    }

    /**
     * This function contains the code for the arcade drive controls.
     */
    private void arcadeDrive() {
        float x;
        float y;
        double driveSpeed;
        double targetDriveSpeed;
        double rawLeft;
        double rawRight;
        double maxPower;

        // Arcade Drive on left stick
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        if (gamepad1.left_stick_button) {
            // Slows Down to Aim if Pressed
            driveSpeed = 5;
        } else {
            // Normal Speed
            driveSpeed = 12.5;
        }
        if (gamepad1.left_trigger_pressed) {
            // Joystick Pressed = Brake
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        } else {
            // Normal Driving
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // This makes sure that the bot is at the same speed at all times
            targetDriveSpeed = 1 * (driveSpeed / ControlHub_VoltageSensor.getVoltage());
            // Cubic Sensitivity = Slower steering when pushed lightly but also can go to the Full capacity
            x = x * x * x;
            y = y * y * y;
            // Everything under is just math to make sure there is proportional steering.
            rawLeft = (y + x) * targetDriveSpeed;
            rawRight = (y - x) * targetDriveSpeed;
            maxPower = Math.max(Math.abs(rawLeft), Math.abs(rawRight));
            if (maxPower > 1) {
                leftDrive.setPower(rawLeft / maxPower);
                rightDrive.setPower(rawRight / maxPower);
            } else {
                leftDrive.setPower(rawLeft);
                rightDrive.setPower(rawRight);
            }
        }
    }
}
