#include <iostream>
#include <memory>
#include <string>
#include <ctime>
//#include <time.h>
#include <fstream>
#include <thread>
#include <list>
#include <mutex>
#include "WPILib.h"
#include "CANTalon.h"
#include "SimPID.h"
#include "AHRS.h"
#include "GripPipeline.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

//CONSTANTS
#define SHOOTER_RPM 4000
#define NATIVE_TO_RPM 0.146484375f
#define GP_L 5
#define GP_R 6
#define DP_UP 0
#define DP_DOWN 180
#define INCHES_TO_ENCODERS 1245/12
#define MIDDLE_PEG_INCHES 69.3
#define INTAKE_SPEED 1.0 //0.6
#define CLIMB_SPEED 80
//#define PRACTICE_BOT

class Robot: public frc::IterativeRobot {
private:
	//variables
	int manipState;
	int fileCount;
	int autoMode;
	int autoState;
	int turnSide;
	int climbState;

	/*int leftT;
	int rightT;
	int center;
	int leftHeight;
	int rightHeight;*/

	/*std::list<int>myList;
	std::mutex myMutex;
	void addToList(int a, int b, int c, i) {

	}*/

	char buffer[50];
	std::ofstream file;

	//Victors
	VictorSP *m_leftDrive0;
	VictorSP *m_leftDrive1;
	VictorSP *m_rightDrive2;
	VictorSP *m_rightDrive3;

	VictorSP *m_intake;
	//VictorSP *m_shooter1;
	//VictorSP *m_agitator;
	//VictorSP *m_elevate;
	VictorSP *m_intoShooter;

	//Talons
	CANTalon *m_shooterB;
	CANTalon *m_shooterA;
	CANTalon *m_climber;
	//CANTalon *//;

	//Encoders
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;

	//navx
	AHRS *nav;

	double last_turn;
	int aim_attempts;

	//Controllers
	Joystick *m_Joystick;
	XboxController *m_Gamepad;

	//pneumatics
	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_gearHoldOut, *m_gearHoldIn;
	Solenoid *m_gearPropOut, *m_gearPropIn;
	Solenoid *m_intakeIn, *m_intakeOut;
	Solenoid *m_introducerIn, *m_introducerOut;
	//PIDS
	SimPID *speedToPowerPID;
	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;

	//LED
	Relay *m_gearLED;
	Relay *m_shotLED;

	//Time
	timeval tv;
	Timer *agTimer;
	Timer *autoTimer;
	Timer *climbTimer;

	static int VisionThread()
	{
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(160,120);
		// camera.SetExposureAuto();
		camera.SetExposureManual(10);
		camera.SetBrightness(65);
		camera.SetFPS(10);
		camera.SetWhiteBalanceAuto();

		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Pipeline", 640, 480);
		cv::Mat source;
		cv::Mat output;
		grip::GripPipeline grip;
		cv::RNG rng(12345);
		cv::Scalar color1 = cv::Scalar( 255,255,255 );
		cv::Scalar color2 = cv::Scalar( 0,255,0 );
		cv::Scalar color3 = cv::Scalar( 255,0,0 );
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Rect> r;
		cv::Mat drawing;
		//char vBuffer[50];
		std::ofstream vFile;
		//vFile.open("/home/lvuser/vision.csv", std::ios::out);

		while(true)
		{
			cvSink.GrabFrame(source);
			grip.Process(source);
			output = *grip.GetHslThresholdOutput();
			drawing = cv::Mat::zeros( source.size(), CV_8UC3 );
			contours = *grip.GetFilterContoursOutput();
			r.resize(contours.size());
			//printf("contour size: %d\n", contours.size());

			for (unsigned int i = 0; i < contours.size(); i ++){
				r[i] = cv::boundingRect(contours[i]);

				cv::drawContours(drawing,
					contours,
					i,
					color1,
					1,
					8,
					std::vector<cv::Vec4i>(),
					0,
					cv::Point());

				rectangle( drawing, r[i].tl(), r[i].br(), color2, 1, cv::LINE_4, 0 );
				//printf("c: %d\tL: %d\tR: %d\n", center, leftHeight, rightHeight);
				//sprintf(vBuffer, "c:%d\tL:%d\tR:%d\n", center, leftHeight, rightHeight);
				//vFile << vBuffer;
			}
			int center = -1;
			if(contours.size() >= 2){
				int leftT  = (r[0].x < r[1].x) ? 0 : 1;
				int rightT = (leftT == 1) ? 0 : 1;
				center 		= r[leftT].x + r[leftT].width + (abs( r[leftT].x + r[leftT].width - r[rightT].x ) >> 1);
				int leftHeight	= r[leftT].height;
				int rightHeight	= r[rightT].height;

				circle( drawing, cv::Point(center, r[leftT].y+(r[leftT].height >> 1)), abs(leftHeight - rightHeight), color3, 1, cv::LINE_4, 0 );
			}
			outputStreamStd.PutFrame(drawing);
			//std::this_thread::__sleep_for(std::chrono::seconds(0), std::chrono::milliseconds(250));
		}
		//	return center;
	}

	void RobotInit(void) override
	{
		//variables
		manipState = 0;
		fileCount = 1;
		autoMode = 0;
		turnSide = 1;
		climbState = 0;

		file.open("/home/lvuser/pid.csv", std::ios::out);

		//Victors
		m_leftDrive0 = new VictorSP(0);
		m_leftDrive0->SetSafetyEnabled(true);
		m_leftDrive1 = new VictorSP(1);
		m_leftDrive1->SetSafetyEnabled(true);
		m_rightDrive2 = new VictorSP(2);
		m_rightDrive2->SetSafetyEnabled(true);
		m_rightDrive3 = new VictorSP(3);
		m_rightDrive3->SetSafetyEnabled(true);

		//m_agitator = new VictorSP(4);
		m_intoShooter = new VictorSP(6);
		//m_elevate = new VictorSP(5);
		m_intake = new VictorSP(4);

		//m_shooter1 = new VictorSP(8);

		//Talons
		m_shooterB = new CANTalon(1);
		m_shooterB->SetControlMode(CANSpeedController::kSpeed);
		m_shooterB->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_shooterB->ConfigEncoderCodesPerRev(4096);
		m_shooterB->SetSensorDirection(true);
		m_shooterB->SetPID(0.04, 0, 0.4, 0.0325);
		m_shooterB->SetCloseLoopRampRate(15);
		//m_shooterB->SetVoltageRampRate(1);
		m_shooterB->SetAllowableClosedLoopErr(0);
		m_shooterB->SelectProfileSlot(0);

		m_shooterA = new CANTalon(3);
		m_shooterA->SetControlMode(CANSpeedController::kFollower);
		m_shooterA->Set(1);

		m_climber = new CANTalon(2);
		m_climber->SetControlMode(CANSpeedController::kSpeed);
		m_climber->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_climber->ConfigEncoderCodesPerRev(4096);
		m_climber->SetSensorDirection(false);
		m_climber->SetPID(0, 0, 0, 2);
		m_climber->SetCloseLoopRampRate(0);
		m_climber->SetAllowableClosedLoopErr(0);
		m_climber->SelectProfileSlot(0);

		// = new CANTalon(2);
		//->SetControlMode(CANSpeedController::kSpeed);

	//	m_shooter12 = new CANTalon(1);

		//encoders
		m_leftEncoder = new Encoder(0,1);
		m_rightEncoder = new Encoder(2,3);

		//navx
#ifndef PRACTICE_BOT
		nav = new AHRS(SPI::Port::kMXP);
#else
		nav = new AHRS(I2C::Port::kOnboard);
#endif

		//vision
		//std::thread visionThread(VisionThread);
		//visionThread.detach();


		last_turn = 0;
		aim_attempts = 0;

		//controllers
		m_Gamepad = new XboxController(1);
		m_Joystick = new Joystick(0);

		//pneumatics
#ifdef PRACTICE_BOT
		m_shiftHigh = new Solenoid(1);
		m_shiftLow = new Solenoid(2);
#else
		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
#endif
		m_gearHoldOut = new Solenoid(2);
		m_gearHoldIn = new Solenoid(3);
		//m_intakeOut = new Solenoid(5);
		//m_intakeIn = new Solenoid();
		m_introducerIn = new Solenoid(4);
		m_introducerOut = new Solenoid(5);

		//LED
		m_gearLED = new Relay(0);
		m_shotLED = new Relay(1);

		//PID
		speedToPowerPID = new SimPID;

		drivePID = new SimPID(0, 0, 0);
		drivePID->setMinDoneCycles(1);
		drivePID->setMaxOutput(0.7);

		turnPID = new SimPID(0, 0, 0);
		turnPID->setMinDoneCycles(1);
		turnPID->setMaxOutput(0.3);

		visionPID = new SimPID(0, 0, 0);
		visionPID->setMinDoneCycles(10);
		visionPID->setMaxOutput(0.4);

		//Time
		agTimer = new Timer();
		agTimer->Reset();
		agTimer->Stop();

		autoTimer = new Timer();
		autoTimer->Reset();
		autoTimer->Stop();

		climbTimer = new Timer();
		climbTimer->Reset();
		climbTimer->Stop();
	}

	void DisabledInit()
	{
		m_gearLED->Set(Relay::kOff);
		m_shotLED->Set(Relay::kOff);
	}

	void DisabledPeriodic()
	{
//#ifndef PRACTICE_BOT
		DriverStation::ReportError("Left encoder" + std::to_string((long)m_leftEncoder->Get()) + "Right Encoder" + std::to_string((long)m_rightEncoder->Get()) + "Gyro" + std::to_string(nav->GetYaw()));
//#endif
		if(m_Joystick->GetRawButton(1)) {
			turnSide = 1;
			DriverStation::ReportError("Turn Side: RED");
		}
		else if(m_Joystick->GetRawButton(2)) {
			turnSide = -1;
			DriverStation::ReportError("Turn Side: BLUE");
		}
	}

	void AutonomousInit()
	{
		autoState = 0;
		nav->Reset();
		m_leftEncoder->Reset();
		m_rightEncoder->Reset();
		m_shooterB->Set(0.f);
		m_gearLED->Set(Relay::kOn);
		autoTimer->Reset();
		//m_shotLED->Set(Relay::kOn);
	}

	void AutonomousPeriodic()
	{
		switch(autoMode) {
		case 0: //still
			m_leftDrive0->SetSpeed(0.f);
			m_leftDrive1->SetSpeed(0.f);
			m_rightDrive2->SetSpeed(0.f);
			m_rightDrive3->SetSpeed(0.f);
			//m_agitator->SetSpeed(0.f);
			m_intake->SetSpeed(0.f);
			m_shooterB->Set(0.f);
			m_intoShooter->SetSpeed(0.f);
			break;
		case 1: //load on middle peg, temporary auto for practice bot
			switch(autoState) {
			case 0: //init
				m_leftDrive0->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
				m_intake->SetSpeed(0.f);
				m_shooterB->Set(0.f);
				m_intoShooter->SetSpeed(0.f);
				autoState++;
				break;
			case 1: //drive to peg
				m_gearHoldOut->Set(false);
				m_gearHoldIn->Set(true);
				if(autoDrive((MIDDLE_PEG_INCHES - 15) * INCHES_TO_ENCODERS, 0)) {
					autoTimer->Start();
					autoState++;
				}
				break;
			case 2: //line up with peg
				if(autoTimer->Get() > 8.0) {
					autoDrive(-1000, 0);
				}
				break;
			}
			break;
		}
	}

	void TeleopInit()
	{
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		m_gearLED->Set(Relay::kOn);
	}

	void TeleopPeriodic()
	{
		operateIntake();
		teleDrive();
		//operateShooter();
		//trim();
		ShooterPID();
		operateShift();
		operateGear();
		advancedClimb();
		//  WHY();
	}

	void TestPeriodic() {

	}

//=====================TELEOP FUNCTIONS=======================

	void operateIntake() {
		if (m_Gamepad->GetRawButton(GP_L)) {
			m_intake->SetSpeed(INTAKE_SPEED);
			//m_elevate->SetSpeed(-INTAKE_SPEED);
		}
		else if(m_Gamepad->GetRawButton(GP_R)) {
			m_intake->SetSpeed(-INTAKE_SPEED);
			//m_elevate->SetSpeed(INTAKE_SPEED);
		}
		else {
			m_intake->SetSpeed(0.f);
			//m_elevate->SetSpeed(0.f);
		}
		/*if(m_Gamepad->GetPOV(DP_UP)) {
			m_intakeIn->Set(true);
			m_intakeOut->Set(false);
		}
		else if(m_Gamepad->GetPOV(DP_DOWN)){
			m_intakeOut->Set(true);
			m_intakeIn->Set(false);
		}*/
	}

	/*void operateShooter()
	{
		if (m_Gamepad->GetAButton())
			m_shooter1->SetSpeed(1.0);
		else
			m_shooter1->SetSpeed(0.0);
	}
*/
	void trim(){
		float motorOutput = 0.5*m_Joystick->GetRawAxis(3) + 0.5;


		m_shooterB->Set(motorOutput);
		float encoderRPM = m_shooterB->GetSpeed()*18.75f/128.f;

		DriverStation::ReportError("Encoder speed" + std::to_string((long)encoderRPM));


	}

	void ShooterPID() {

		//int setPoint = -(1500 * (0.5 * m_Joystick->GetRawAxis(3) + 0.5) + 3000);
		int setPoint = -3200;
		gettimeofday(&tv, 0);

		float encoderRPM = m_shooterB->GetSpeed();
		DriverStation::ReportError("setpoint: "+ std::to_string(setPoint) + "Encoder speed" + std::to_string((long)encoderRPM));

		if(m_Gamepad->GetAButton()) {
			agTimer->Start();
			m_shooterB->Set(setPoint);
			//->Set(SHOOTER_RATIO);

			if(agTimer->Get() > 0.8)
				m_intoShooter->SetSpeed(1.0);
			if(agTimer->Get() > 3.8) {
				m_introducerOut->Set(false);
				m_introducerIn->Set(true);
			}

			DriverStation::ReportError("speed error " + std::to_string(m_shooterB->GetClosedLoopError()*NATIVE_TO_RPM));

			sprintf(buffer, "%d:%d , %d , %d , %f\n", (int)tv.tv_sec, (int)tv.tv_usec, setPoint, (int)encoderRPM, m_shooterB->GetClosedLoopError()*NATIVE_TO_RPM);
			file << buffer;

			fileCount++;
		}
		else if(m_Gamepad->GetStartButton()) {
			m_shooterB->Set(0.3 * SHOOTER_RPM);
			m_intoShooter->SetSpeed(-0.2);
		}
		else if(m_Gamepad->GetBButton())
			m_intoShooter->SetSpeed(0.5);
		else
		{
			m_shooterB->Set(0.f);
			//->Set(0.f);
			m_intoShooter->SetSpeed(0.f);
			m_introducerOut->Set(true);
			m_introducerIn->Set(false);
			agTimer->Stop();
			agTimer->Reset();
		}

		/*if(m_Gamepad->GetBButton()){
			m_intoShooter->SetSpeed(0.5);
		}
		else if (!m_Gamepad->GetBButton()){
			m_intoShooter->SetSpeed(0.f);
		}*/
	}
	/*void WHY() {
		if(m_Gamepad->GetAButton()) {
			m_shooter1->Set(0.30);
		}
		else  {
			m_shooter1->Set(0.f);
		}
		//else if(m_Gamepad->GetBButton()) {
		//	m_intoShooter->SetSpeed(0.5);
		//}
		//else {
			//m_shooter1->SetSpeed(0.f);
		//}
	}*/
#define PRACTICE_DRIVE_LIMIT 1

	inline void teleDrive(void) {
		float leftSpeed = scale(limit(expo(-m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.8f), PRACTICE_DRIVE_LIMIT);
    	float rightSpeed = scale(-limit(expo(-m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.8f), PRACTICE_DRIVE_LIMIT);

		m_leftDrive0->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
	}

/*	void manipulatorControl(void) {

		switch(manipState)
		{
		case 0:
			//m_agitator->SetSpeed(0);
			//m_shooter1->Set(0);
			////->Set(0);

			if(m_Gamepad->GetAButton())
			{
				manipState = 1;
			}

			break;

		case 1:
			//m_agitator->SetSpeed(0);
			//m_shooter1->Set(0);
			////->Set(0);

			if(m_Gamepad->GetBButton())
			{
				manipState = 2;
			}

			break;

		case 2:
			//m_agitator->SetSpeed(1.0);
			//m_shooter1->Set(1.0);
			////->Set(1.0);

			if(m_Gamepad->GetXButton())
			{
				manipState = 1;
			}

			break;
		}
	}*/

	void operateShift() {
		if(m_Joystick->GetRawButton(1)) {
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else {
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}

	void operateGear() {
		if(m_Gamepad->GetYButton()) {
			m_gearHoldIn->Set(true);
			m_gearHoldOut->Set(false);
		}
		else if(m_Gamepad->GetXButton()) {
			m_gearHoldIn->Set(false);
			m_gearHoldOut->Set(true);
		}
	}

	void advancedClimb() {
		//printf("climb error: %d\n", m_climber->GetClosedLoopError()*NATIVE_TO_RPM);
		if(m_Gamepad->GetPOV(0) == DP_UP) {
			m_climber->Set(CLIMB_SPEED);
			//if(m_climber->GetSpeed() < 10)
				//m_climber->Set(0.f);
		}
		else if(m_Gamepad->GetPOV(0) == DP_DOWN)
			m_climber->Set(-CLIMB_SPEED);
		else
			m_climber->Set(0.f);
	}

//=====================VISION FUNCTIONS=====================

	/*bool aimAtTarget() {
		float turn = last_turn;

		return 0;
	}*/

	bool lineUpGear() {//travis did this :3
		int targetCenter;
		return false;
	}

//=====================AUTO FUNCTIONS=====================

	bool autoDrive(int distance, int angle) {
		int currentDist = (m_rightEncoder->Get() + m_leftEncoder->Get()) / 2;
		int currentAngle = nav->GetYaw();

		drivePID->setDesiredValue(distance);
		turnPID->setDesiredValue(angle);

		float drive = -drivePID->calcPID(currentDist);
		float turn = -turnPID->calcPID(currentAngle);

		m_leftDrive0->SetSpeed(limit(drive + turn, 1));
		m_leftDrive1->SetSpeed(limit(drive + turn, 1));
		m_rightDrive2->SetSpeed(-limit(drive - turn, 1));
		m_rightDrive3->SetSpeed(-limit(drive - turn, 1));

		return drivePID->isDone() && turnPID->isDone();
	}

//=======================MATHY FUNCTIONS============================
	/*float limit(float s) {
		if (s > 1)
			return 1;
		else if (s < -1)
			return -1;
		return s;
	}*/

	inline float expo(float x, int n)
	{
		int sign = n % 2;
		float y = 1;
		for(int i = 1; i <= n; i++)
		{
			y *= x;
		}
		if(sign == 0 && x < 0)
			return -y;
		return y;
	}

	inline float limit(float x, float lim)
	{
		if(x > lim)
			return lim;
		else if(x < -lim)
			return -lim;
		return x;
	}

	inline float scale(float x, float scale)
	{
		return x * scale;
	}
};
	/*Void DONOTENTER() {
his name is travis -Bacca
}*/
START_ROBOT_CLASS(Robot)
