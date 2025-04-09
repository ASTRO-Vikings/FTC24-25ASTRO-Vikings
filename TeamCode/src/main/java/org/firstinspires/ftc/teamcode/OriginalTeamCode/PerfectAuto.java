package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous(name="Auto :)", group ="AHHHHHHHH", preselectTeleOp = "Teleop To Use :))))")
public class PerfectAuto extends LinearOpMode {

    public START_POSITION startPosition;
    Limelight3A limelight;
    final double OPEN = 0.75;
    final double CLOSE = 0.4;
    final double ARMROTMULT = 1;

    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
//    CRServo spool = null;
    ServoImplEx wrist = null;
    Servo grabber = null;


    double actuatorPos = 0;
    double armRotPos = 0;
    double wristRotPos=0;

    SampleMecanumDrive drive;


    void controlLinearActuator() {
        linearActuator.setTargetPosition((int) actuatorPos);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void controlGrabber(double i) {
        grabber.setPosition(i);
    }

    double armPos = 0;

    void controlBothArmExtenders() {
        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition(-(int) armPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void controlArmRotate() {
        armRotate.setTargetPosition((int) (armRotPos * ARMROTMULT));
    }

    void controlWristRotate(){
        wrist.setPosition(wristRotPos);
    }

    public void selectStartingPosition() {

        //******select start pose***);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            while(!isStopRequested())
            {
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                return;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                return;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                return;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                return;
            }
            telemetry.update();
            }
        }
    

    void resetPosWithLL(Pose2d defaultPose){
        Pose2d curPose =  defaultPose;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                curPose = new Pose2d(new Vector2d(botpose.getPosition().x*39.37,botpose.getPosition().y*39.37),botpose.getOrientation().getYaw(AngleUnit.RADIANS));
                drive.setPoseEstimate(curPose);
            }
        }

    }

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        armLifterLeft = hardwareMap.dcMotor.get("armLifterLeft");
        armLifterRight = hardwareMap.dcMotor.get("armLifterRight");
        armRotate = hardwareMap.dcMotor.get("armRotate");
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        grabber = hardwareMap.servo.get("grabber");


        armLifterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double slowerVelocity = 25; // 25 is half ish of the full speed
        TrajectoryVelocityConstraint velCons = SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accCons = SampleMecanumDrive.getAccelerationConstraint(slowerVelocity);

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
        armRotate.setTargetPosition((int) (armRotPos * ARMROTMULT));
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }

        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        LLResult result = limelight.getLatestResult();
        telemetry.addData("", "got latest result");
        if (result != null) {
            telemetry.addData("", "result not null");
            if (result.isValid()) {
                telemetry.addData("", "result is valid");
            }
        }
        Pose2d startPose =  new Pose2d(0, 0, 0);
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                startPose = new Pose2d(new Vector2d(botpose.getPosition().x*39.37,botpose.getPosition().y*39.37),botpose.getOrientation().getYaw(AngleUnit.RADIANS));                telemetry.addData("Start pose", startPose.toString());
            }
        }
        // Push telemetry to the Driver Station.

        telemetry.update();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        //hang spec
        Trajectory tr1 = null;
        Trajectory tr2 = null;
        Trajectory tr3 = null;
        Trajectory tr4 = null;
        Trajectory tr5 = null;
        Trajectory tr6 = null;
        Trajectory tr7 = null;
        Trajectory tr8 = null;
        Trajectory tr9 = null;
        switch (startPosition) {
            case BLUE_LEFT:
            tr1 = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(10, 38,Math.toRadians(270)), 0)
                    .build();
            tr2 = drive.trajectoryBuilder(tr1.end())
                    .back(16)
                    .build();
            //pick up 1st samp
            tr3 = drive.trajectoryBuilder(tr2.end())
                    .splineToLinearHeading(new Pose2d(50, 48.9,Math.toRadians(-84.57)), 5)
                    .build();
            //drop off
            tr4 = drive.trajectoryBuilder(tr3.end())
                    .splineToLinearHeading(new Pose2d(68, 63, Math.toRadians(68)), 0)
                    .build();
            //fix self
            tr5 = drive.trajectoryBuilder(tr4.end())
                    .splineToLinearHeading(new Pose2d(58, 60, Math.toRadians(270)), 0)
                    .build();
            //pick up 2nd sample
            tr6 = drive.trajectoryBuilder(tr5.end())
                    .splineToConstantHeading(new Vector2d(65, 48.9), 0)
                    .addTemporalMarker(.05, () -> {
                        armRotPos = -651;
                        wristRotPos = 0.55;
                        armPos = 40;
                        controlBothArmExtenders();
                        controlArmRotate();
                        controlWristRotate();
                    })
                    .build();
            //drop off
            tr7 = drive.trajectoryBuilder(tr6.end())
                    .splineToLinearHeading(new Pose2d(67, 63, Math.toRadians(67.9)), 0)
                    .build();
            //line up
            tr8 = drive.trajectoryBuilder(tr7.end())
                    .back(8)
                    .splineToSplineHeading(new Pose2d(40, 14, Math.toRadians(0)), 0)
                    .addTemporalMarker(0.6, () -> {
                        controlArmRotate();
                        controlWristRotate();
                        controlBothArmExtenders();
                    })
                    .build();
            //lvl 1 ascent/park
            tr9 = drive.trajectoryBuilder(tr8.end())
                    .splineToConstantHeading(new Vector2d(32, 14), 0)
                    .addTemporalMarker(.001, () -> {
                        armPos = -3400;
                        controlBothArmExtenders();
                    })
                    .build();
            break;
            case RED_LEFT://---------------------------------------------------------------------------
                tr1 = drive.trajectoryBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(-10, -46,Math.toRadians(270+180)), 0)
                        .build();
                tr2 = drive.trajectoryBuilder(tr1.end())
                        .back(14)
                        .build();
                //pick up 1st samp
                tr3 = drive.trajectoryBuilder(tr2.end())
                        .splineToLinearHeading(new Pose2d(-50, -50.9,Math.toRadians(-84.57+180)), 5)
                        .build();
                //drop off
                tr4 = drive.trajectoryBuilder(tr3.end())
                        .splineToLinearHeading(new Pose2d(-68.5, -66.5, Math.toRadians(68+180)), 0)
                        .build();
                //fix self
                tr5 = drive.trajectoryBuilder(tr4.end())
                        .splineToLinearHeading(new Pose2d(-58, -65, Math.toRadians(270+180)), 0)
                        .build();
                //pick up 2nd sample
                tr6 = drive.trajectoryBuilder(tr5.end())
                        .splineToConstantHeading(new Vector2d(-65, -50.9), 0)
                        .addTemporalMarker(.2, () -> {
                            armRotPos = -651;
                            wristRotPos = 0.576;
                            armPos = 100;
                            controlBothArmExtenders();
                            controlArmRotate();
                            controlWristRotate();
                        })
                        .build();
                //drop off
                tr7 = drive.trajectoryBuilder(tr6.end())
                        .splineToLinearHeading(new Pose2d(-67.5, -66.5, Math.toRadians(67.9+180.2)), 0)
                        .build();
                //line up
                tr8 = drive.trajectoryBuilder(tr7.end())
                        .back(8)
                        .splineToSplineHeading(new Pose2d(-40, -19, Math.toRadians(0+180)), 0)
                        .addTemporalMarker(1, () -> {
                            controlArmRotate();
                            controlWristRotate();
                            controlBothArmExtenders();
                        })
                        .build();
                //lvl 1 ascent/park
                tr9 = drive.trajectoryBuilder(tr8.end())
                        .splineToConstantHeading(new Vector2d(-26, -19), 0)
                        .addTemporalMarker(.001, () -> {
                            armPos = -3400;
                            controlBothArmExtenders();
                        })
                        .build();
        }
//
        waitForStart(); /*****  DON'T RUN ANY MOTOR MOVEMENT ABOVE THIS LINE!! You WILL get PENALTIES! And it's UNSAFE! *****/
        if (isStopRequested()) return;
//
//        /***** start of manual code running or initiation or whatever *****/
        controlGrabber(CLOSE);
        armRotPos = -2552.9;
        wristRotPos = 0.65;
        controlWristRotate();
        controlArmRotate();
        sleep(600);
        drive.followTrajectory(tr1);
        wristRotPos = 0.63;
        armRotPos = -2532.9;
        controlArmRotate();
        controlWristRotate();
        sleep(100);
//        resetPosWithLL(drive.getPoseEstimate());
        drive.followTrajectory(tr2);
        controlGrabber(OPEN);
        sleep(200);
//        resetPosWithLL(drive.getPoseEstimate());
        drive.followTrajectory(tr3);
        armRotPos = -455;
        wristRotPos = 0.55;
        controlArmRotate();
        controlWristRotate();
        sleep(750);
        controlGrabber(CLOSE); //first sample
        sleep(600);
        armRotPos = -2838;
        wristRotPos = 0.57;
        armPos = -1883;
        controlWristRotate();
        controlArmRotate();
        controlBothArmExtenders();
        sleep(350);
//        resetPosWithLL(drive.getPoseEstimate());
        drive.followTrajectory(tr4);
        sleep(100);
        controlGrabber(OPEN);
        sleep(200);
        drive.followTrajectory(tr5);
//        sleep(300);
        drive.followTrajectory(tr6);
        sleep(700);
        controlGrabber(CLOSE); //2nd sample
        sleep(600);
        armRotPos = -2964.3;
        wristRotPos = 0.65;
        armPos = -4732.1;
        controlWristRotate();
        controlArmRotate();
        controlBothArmExtenders();
        sleep(250);
        drive.followTrajectory(tr7);
        controlGrabber(OPEN);
        sleep(300);
        armPos = 0;
//        controlBothArmExtenders();
//        armPos = -3400;
        wristRotPos = 0.4;
        armRotPos = -100;
        drive.followTrajectory(tr8);
        drive.followTrajectory(tr9);


        /***** end of manual code running or initiation or whatever *****/
    }



    public Vector2d poseToVector(Pose2d Pose) {
        return new Vector2d(Pose.getX(), Pose.getY());
    }
    public Pose2d vectorToPose(Vector2d Vector, Double Heading) {
        return new Pose2d(Vector.getX(), Vector.getY(), Math.toRadians(Heading));
    }

    public Pose2d vectorToPose(Vector2d Vector, int Heading) {
        return new Pose2d(Vector.getX(), Vector.getY(), Math.toRadians(Heading));
    }

}