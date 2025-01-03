package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name="Teleop To Use :))))",group = "Teleops to use :))))))")
public class FirstRealTeleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    final int ASCENT_UP = 14200;
    final int ASCENT_DOWN = 2200;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
//    CRServo spool = null;
    Servo wrist = null;
    CRServo sampPickUpLeft = null;
    CRServo sampPickUpRight = null;


    double actuatorPos = 0;
    double armRotPos = 0;
    double wristPos = 1;
    boolean canToggle = true;

    void linearActuatorMover() {
        linearActuator.setTargetPosition((int) actuatorPos);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void controlBothGrabbers(int i) {
        sampPickUpLeft.setPower(i);
        sampPickUpRight.setPower(i);
    }

    double armPos = 0;

    void controlBothArmExtenders() {
        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition(-(int) armPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() {
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(),new ArrayList<>());
        localizer.setPoseEstimate(new Pose2d());

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        armLifterLeft = hardwareMap.dcMotor.get("armLifterLeft");
        armLifterRight = hardwareMap.dcMotor.get("armLifterRight");
        armRotate = hardwareMap.dcMotor.get("armRotate");
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
//        spool = hardwareMap.crservo.get("spool");
        wrist = hardwareMap.servo.get("wrist");
        sampPickUpLeft = hardwareMap.crservo.get("sampPickUpLeft");
        sampPickUpRight = hardwareMap.crservo.get("sampPickUpRight");

        sampPickUpLeft.setDirection(CRServo.Direction.FORWARD);
        sampPickUpRight.setDirection(CRServo.Direction.REVERSE);

        armLifterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Timer timer = new Timer();
        long delay = 250L;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();
        runtime.reset();

        boolean isGrabbing = false;
        armLifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLifterRight.setPower(1);
        armLifterLeft.setPower(1);
        linearActuator.setPower(1);
        armRotate.setPower(1);

        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition((int) armPos);
        linearActuator.setTargetPosition((int) actuatorPos);
        armRotate.setTargetPosition((int) armRotPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            localizer.update();
            linearActuatorMover();
            controlBothArmExtenders();
            double pose = drive.getExternalHeading(); // used for my custom function to drive straight thru the trusses

            double slowMode = gamepad1.left_trigger;
            double slowCoeff = 0.3;

            double fastMode = gamepad1.right_trigger;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if (slowMode > 0.2) {

                leftFrontDrive.setPower(leftFrontPower * slowCoeff);
                rightFrontDrive.setPower(rightFrontPower * slowCoeff);
                leftBackDrive.setPower(leftBackPower * slowCoeff);
                rightBackDrive.setPower(rightBackPower * slowCoeff);

            } else if (fastMode > 0.2) {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            } else {
                leftFrontDrive.setPower(leftFrontPower * 0.7);
                rightFrontDrive.setPower(rightFrontPower * 0.7);
                leftBackDrive.setPower(leftBackPower * 0.7);
                rightBackDrive.setPower(rightBackPower * 0.7);

            }

//            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            armRotate.setPower(gamepad2.left_stick_y);
//            wrist.setPosition(-(gamepad2.right_stick_y));
//
//            armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            wrist.setPosition(wristPos);
//            wristPos += (Math.pow(gamepad2.right_stick_y,3));
//
//            if(gamepad2.dpad_up){
//                linearActuatorRaiser();
//            } else if(gamepad2.dpad_down){
//                linearActuatorLower();
//            }
//
//            int spoolPow = 0;
//            if(gamepad2.dpad_up){
//                spoolPow = 1;
//            } else if(gamepad2.dpad_down){
//                spoolPow = -1;
//            }
//
//            armLifterLeft.setPower(gamepad2.right_stick_x);
//            armLifterRight.setPower(gamepad2.right_stick_x);
//
//
//            if(isGrabbing){
//                sampPickUpRight.setPower(1);
//                sampPickUpLeft.setPower(1);
//            } else{
//                sampPickUpRight.setPower(0);
//                sampPickUpLeft.setPower(0);
//            }
//            if (gamepad2.left_bumper){
//                isGrabbing = !isGrabbing;
//            }
//
//            if(gamepad2.right_bumper){
//                sampPickUpLeft.setPower(-1);
//                sampPickUpRight.setPower(-1);
//            }
//            spool.setPower(spoolPow);

            //arm rotate
            armRotate.setTargetPosition((int) armRotPos);
            double armRotChange = gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y * 75;
            if (armRotate.getCurrentPosition() + armRotChange > -2200) {
                armRotPos += armRotChange;
            }
            if (armRotate.getCurrentPosition() < -2200) {
                armRotate.setPower(0.25);
                armRotPos = -2100;
            } else {
                armRotate.setPower(1);
            }
            //actuator
            if (gamepad2.dpad_up) {
                actuatorPos = ASCENT_UP;
//                linearActuator.setPower(1);
            } else if (gamepad2.dpad_down) {
                actuatorPos = ASCENT_DOWN;
//                linearActuator.setPower(-1);
            }// else {
//                linearActuator.setPower(0);
//            }

            //extenders
            double rightTrig = gamepad2.right_trigger;
            double leftTrig = gamepad2.left_trigger;
            if (armPos - rightTrig * 75 > -3010) {
                armPos -= rightTrig * 75;
            }
            if (armPos + leftTrig * 75 < 0) {
                armPos += leftTrig * 75;
            }
//            armLifterLeft.setPower(gamepad2.right_stick_y);
//            armLifterRight.setPower(gamepad2.right_stick_y);

//            //wrist
//            if(gamepad2.dpad_left){
//                wristPos = 0.5;}
            if (gamepad2.right_stick_y > 0.1 && wristPos < 1) {
                wristPos += 0.01;
            } else if (gamepad2.right_stick_y < -0.1 && wristPos > 0) {
                wristPos -= 0.01;
            }
            wrist.setPosition(wristPos);

            //grabber
            if (canToggle && gamepad2.x) {
                TimerTask task = new TimerTask() {
                    @Override
                    public void run() {
                        canToggle = true;
                        controlBothGrabbers(0);
                    }
                };
                canToggle = false;
                controlBothGrabbers(1);
                timer.schedule(task, delay);

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pose", "Pose: " + localizer.getPoseEstimate());
            telemetry.addData("Arm height", "Arm height: " + actuatorPos);
            telemetry.addData("Arm rot", "Arm rot: " + armRotPos);
            telemetry.addData("wrist rot", "Wrist rot: " + wristPos);
            telemetry.update();
        }
    }
}