package org.firstinspires.ftc.teamcode.bots;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotV3 extends MecanumDrive{
    final double ALLOWED_ERROR = 0.1;
    ServoImplEx angle,pusher,arm,grip,leftIntakeLinkage,rightIntakeLinkage, drone;
    DcMotorEx leftSlides,rightSlides,intake;
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    AnalogInput getAngle,getPusherPosition,getArmPosition,getGripPosition,leftIntakeLinkagePosition,rightIntakeLinkagePosition;
    //RevBlinkinLedDriver blinkinLedDriverLeft,blinkinLedDriverRight;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    RevTouchSensor pixel1,pixel2;
    public double p,i,d,f,Target;
    public double p1,i1,d1,f2;
    private PIDController Controller;
    private PIDController RobotController;
    int pixels;
    double linearSlidePosition, currentAngle, currentArmPosition, currentGripPosition, clawPusherPosition, calculatedPower;
    double linearSlidePower = f;
    ElapsedTime atTarget = new ElapsedTime();

    public RobotV3(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        //hardware map
        angle = hardwareMap.get(ServoImplEx.class,"claw_angle");
        arm = hardwareMap.get(ServoImplEx.class,"claw_arm");
        pusher = hardwareMap.get(ServoImplEx.class,"claw_pusher");
        grip = hardwareMap.get(ServoImplEx.class,"claw_grip");
        leftIntakeLinkage = hardwareMap.get(ServoImplEx.class,"left_intake_linkage");
        rightIntakeLinkage = hardwareMap.get(ServoImplEx.class,"right_intake_linkage");
        drone = hardwareMap.get(ServoImplEx.class,"drone");
        //pwm ranges
        angle.setPwmRange(new PwmControl.PwmRange(510,2490));
        arm.setPwmRange(new PwmControl.PwmRange(510,2490));
        grip.setPwmRange(new PwmControl.PwmRange(510,2490));
        pusher.setPwmRange(new PwmControl.PwmRange(1100,2150));
        leftIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        rightIntakeLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        drone.setPwmRange(new PwmControl.PwmRange(1000,2000));
        //set direction
        pusher.setDirection(Servo.Direction.REVERSE);
        rightIntakeLinkage.setDirection(Servo.Direction.REVERSE);
        drone.setDirection(Servo.Direction.REVERSE);
        //SERVOS
        //MOTORS
        //hardware map
        leftSlides = hardwareMap.get(DcMotorEx.class,"left_slides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"right_slides");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        //encoder
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class, "left_led");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class, "right_led");
        //set direction
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //MOTORS
        //SENSORS
        pixel1 = hardwareMap.get(RevTouchSensor.class,"pixel1");
        pixel2 = hardwareMap.get(RevTouchSensor.class,"pixel2");
        getAngle = hardwareMap.get(AnalogInput.class,"claw_angle_position");
        getArmPosition = hardwareMap.get(AnalogInput.class,"claw_arm_position");
        getPusherPosition = hardwareMap.get(AnalogInput.class,"claw_pusher_position");
        getGripPosition = hardwareMap.get(AnalogInput.class,"claw_grip_position");
        leftIntakeLinkagePosition = hardwareMap.get(AnalogInput.class,"left_intake_linkage_position");
        rightIntakeLinkagePosition = hardwareMap.get(AnalogInput.class,"right_intake_linkage_position");
        p=2.5;i=0;d=0;f=0;Target = 0;
        p1=0.04;i1=0.0;d1=0;
        Controller = new PIDController(p,i,d);
        RobotController = new PIDController(p1,i1,d1);
    }

    public void pidTuning(double p, double i, double d, double f, double Target){
        this.p1 = p;
        this.i1 = i;
        this.d1 = d;
        this.f = f;
        this.Target = Target;
        RobotController.setPID(p,i,d);
    }

    public void updateRobotState(){
        pixels = (int)pixel1.getValue()+(int)pixel2.getValue();
        linearSlidePosition = ((intake.getCurrentPosition()/8192.0)+(leftSlides.getCurrentPosition()/537.7)+(rightSlides.getCurrentPosition()/537.7))/3;
        currentAngle = Math.abs(((getAngle.getVoltage()/3.3)*360)-348);
        currentArmPosition=Math.abs(((getArmPosition.getVoltage()/3.3)*360)-275);
        currentGripPosition=((getGripPosition.getVoltage()/3.3)*360);
        clawPusherPosition=((getPusherPosition.getVoltage()/3.3)*360);
        calculatedPower = Controller.calculate(linearSlidePosition, Target);
        updatePoseEstimate();
        if(!slidesWithinRange(ALLOWED_ERROR)) atTarget.reset();
        linearSlidePower = f;
    }
    public double getError(){
        return Math.abs(Target-linearSlidePosition);
    }
    public boolean slidesWithinRange(double allowedError){
        return getError() <= allowedError;
    }
    public void setLinearSlidePower(double power){
        linearSlidePower = power;
    }

    public double getCalculatedPower() {
        return calculatedPower;
    }
    public void slideZeroCondition(double zero, double power){
        if(zero == Target) linearSlidePower -= power;
    }
    public void closeIntake(){
        leftIntakeLinkage.setPosition(.1);
        rightIntakeLinkage.setPosition(.1);
    }
    public void downIntake(){
        leftIntakeLinkage.setPosition(1);
        rightIntakeLinkage.setPosition(1);
    }
    public void topIntake(){
        leftIntakeLinkage.setPosition(0.8);
        rightIntakeLinkage.setPosition(0.8);
    }
    public void middleIntake(){
        leftIntakeLinkage.setPosition(0.92);
        rightIntakeLinkage.setPosition(0.92);
    }
    public void primeIntake(){
        leftIntakeLinkage.setPosition(0.5);
        rightIntakeLinkage.setPosition(0.5);
    }
    public void activateSlides(){
        leftSlides.setPower(linearSlidePower);
        rightSlides.setPower(linearSlidePower);
    }

    public void setTarget(double target) {
        Target = target;
    }
    public void setGripPosition(double position){
        grip.setPosition(position);
    }

    public int getPixels() {
        return pixels;
    }
    public void setIntakePower(double power){
        intake.setPower(power);
    }

    public double getLinearSlidePosition() {
        return linearSlidePosition;
    }
    public void incrementLinearSlideTarget(double increment){
        Target += increment;
    }
    public void setArmPosition(double position){
        arm.setPosition(position);
    }
    public void setAnglePosition(double position){
        angle.setPosition(position);
    }

    public ServoImplEx getArm() {
        return arm;
    }
    public void setPusherPosition(double position){
        pusher.setPosition(position);
    }

    public double getCurrentArmPosition() {
        return currentArmPosition;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public DcMotorEx getIntake() {
        return intake;
    }

    public DcMotorEx getRightSlides() {
        return rightSlides;
    }

    public DcMotorEx getLeftSlides() {
        return leftSlides;
    }

    public double getTarget() {
        return Target;
    }

    public PIDController getController() {
        return Controller;
    }

    public double getTimeAtTarget() {
        return atTarget.milliseconds();
    }

    public void setBothLED(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriverLeft.setPattern(pattern);
        blinkinLedDriverRight.setPattern(pattern);
    }
    public void setRightLED(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriverRight.setPattern(pattern);
    }
    public void setLeftLED(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriverLeft.setPattern(pattern);
    }
    public void shootDrone(){
        drone.setPosition(1);
    }
    public void setDronePosition(double position){
        drone.setPosition(position);
    }
    public void resetSlideEncoders(){
        rightSlides.setMode(STOP_AND_RESET_ENCODER);
        rightSlides.setMode(RUN_WITHOUT_ENCODER);
        leftSlides.setMode(STOP_AND_RESET_ENCODER);
        leftSlides.setMode(RUN_WITHOUT_ENCODER);
        intake.setMode(STOP_AND_RESET_ENCODER);
        intake.setMode(RUN_WITHOUT_ENCODER);
    }

    public double centerBasedOnYaw(double yaw){
        double power = RobotController.calculate(yaw, 0);
        if(Math.abs(yaw)>1){
            leftFront.setPower(leftFront.getPower()+power);
            leftBack.setPower(leftBack.getPower()+power);
            rightFront.setPower(rightFront.getPower()-power);
            rightBack.setPower(rightBack.getPower()-power);
        }
        return power;
    }

    public void strafeLeft(){
        leftFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(-0.5);
    }
    public void strafeRight(){
        leftFront.setPower(0.5);
        leftBack.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightBack.setPower(0.5);
    }


}
