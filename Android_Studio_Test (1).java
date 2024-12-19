package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Go")
public class Android_Studio_Test extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor BL_Motor;
    public DcMotor BR_Motor;
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor UPPArm_Motor;
    public DcMotor Vertical_Rack;

    public Servo Grabber_Servo;

    public int maxVerticalRackHeight = 1818;
    public int minVerticalRackHeight = 0;

    public double Rack_Pos = 0.0;
    public double Rack_inc = 0.05;
    public boolean dpad_latch_right = false;
    public boolean dpad_latch_left = false;

    @Override
    public void runOpMode() {
        BR_Motor = hardwareMap.get(DcMotor.class, "BR Wheel Motor"); //deviceName 3
        FL_Motor = hardwareMap.get(DcMotor.class, "FL Wheel Motor"); //deviceName 2
        BL_Motor = hardwareMap.get(DcMotor.class, "BL Wheel Motor"); //deviceName 1
        FR_Motor = hardwareMap.get(DcMotor.class, "FR Wheel Motor"); //deviceName 0
        UPPArm_Motor = hardwareMap.get(DCMotor.class, "Upper Arm Motor"); //we need the deviceName
        Vertical_Rack = hardwareMap.get(DcMotor.class, "Vertical Rack"); //0 on expansion hub

        Grabber_Servo = hardwareMap.get(Servo.class, "Grabber_Servo"); //we need the deviceName

        Vertical_Rack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Vertical_Rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean intakeIn = false;
        boolean grabbing = false;

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //double Horizontal = gamepad1.right_trigger + -gamepad1.left_trigger;
            //double DriveY = -(gamepad1.right_stick_y + gamepad1.left_stick_y) / 2;
            //double DriveX = (gamepad1.right_stick_x + gamepad1.left_stick_x) / 2;

            double axial =gamepad1.left_stick_Y;
            double lateral = gamepad1.left_stick_X;
            double yaw = gamepad1.right_stick_X;

            double leftFrontPower = axial + Lateral + yaw;
            double rightFrontPower = axial - Lateral - yaw;
            double leftBackPower = axial - Lateral + yaw;
            double rightBackPower = axial + Lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.abs(leftFrontPower),Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            FL_Motor.setPower(leftFrontPower);
            FR_Motor.setPower(rightFrontPower);
            BL_Motor.setPower(leftBackPower);
            BR_Motor.setPower(rightBackPower);

            // Crane motion idk they did this so not me // Crane motion idk they did this so not me // Crane motion idk they did this so not me //

            double verticalRackSpeed = (gamepad2.right_trigger - gamepad2.left_trigger);

//            if (Vertical_Rack.getCurrentPosition() < minVerticalRackHeight) {
//                Vertical_Rack.setPower(1);
//            } else if (Vertical_Rack.getCurrentPosition() > maxVerticalRackHeight) {
//                Vertical_Rack.setPower(-1);
//            } else {
                Vertical_Rack.setPower(verticalRackSpeed);
//            }



//            Rack_Motor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
//            Rack_Servo.setPower(gamepad2.right_stick_y);

            // Spinning Intake // Spinning Intake // Spinning Intake // Spinning Intake // Spinning Intake // Spinning Intake // Spinning Intake //

            if (gamepad2.y) {
                grabbing = true;
            } else if (gamepad2.x) {
                grabbing = false;
            }

            if (grabbing) {
                Grabber_Servo.setPosition(0);
            } else {
                Grabber_Servo.setPosition(1);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Rack Position", Vertical_Rack.getCurrentPosition());
            telemetry.update();
        }
    }
}

