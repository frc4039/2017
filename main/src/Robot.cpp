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
#include "shiftlib.h"

//CONSTANTS
#define SHOOTER_RPM 4000
#define NATIVE_TO_RPM 0.146484375f
#define GP_L 5
#define GP_R 6
#define DP_UP 0
#define DP_DOWN 180
#define GP_BL 2
#define GP_BR 3
#define INCHES_TO_ENCODERS 1245/12
#define MIDDLE_PEG_INCHES 69.3
#define INTAKE_SPEED 1.0 //0.6
#define CLIMB_SPEED 80
//#define SHOOTER_SPEED -3160 //practice bot
#define SHOOTER_SPEED -3065
//#define AUTO_SHOOTER_SPEED -3325 //practice bot
#define AUTO_SHOOTER_SPEED -3325
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
	int lastClimberPos;

	/*int leftT;
	int rightT;
	int center;
	int leftHeight;
	int rightHeight;*/

	/*std::list<int>myList;
	std::mutex myMutex;
	void addToList(int a, int b, int c, i) {

	}*/
//======================PathFollow Variables=================
	PathFollower *CLAMPS;

	Path *path_gearCenterPeg, *path_gearCenterPegBlue2;
	Path *path_gearLeftPeg, *path_gearLeftPeg2;
	Path *path_gearRightPeg, *path_gearRightPeg2;

	Path *path_rightShot1;
	Path *path_rightShot2;
	Path *path_leftShot1;
	Path *path_leftShot2;

	SimPID *pathDrivePID;
	SimPID *pathTurnPID;

	float leftSpeed, rightSpeed;
	int setPoint;

	char buffer[50];
	std::ofstream file;

	//Victors
	VictorSP *m_leftDrive0;
	VictorSP *m_leftDrive1;
	VictorSP *m_rightDrive2;
	VictorSP *m_rightDrive3;

	VictorSP *m_intake;
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
	Solenoid *m_gearPushOut, *m_gearPushIn;
	Solenoid *m_gearPropOut, *m_gearPropIn;
	Solenoid *m_intakeIn, *m_intakeOut;
	Solenoid *m_introducerIn, *m_introducerOut;
	Solenoid *m_gearHoldIn, *m_gearHoldOut;

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
	float agLastTime;
	Timer *autoTimer;
	Timer *climbTimer;

	enum TurnSide { RED_SIDE = 1, BLUE_SIDE = -1 };

	//vision
	//static float pegX, pegY, pegAngle;

#define KNOWN_HEIGHT 5.0f
#define KNOWN_WIDTH 10.25f
#define FOCAL_POINT 120.f
#define PEG_OFFSET_X 0.0f
#define PEG_OFFSET_Y 11.0f
#define HFOV 0.449422282f
#define HRES 160.0f
#define VRES 120.0f
#define INCH_TO_ENC 5500.f/60.f
#define rad2deg 180.f/PI

	static inline float getDistance(int pixelHeight){
		return KNOWN_HEIGHT * FOCAL_POINT / (float)pixelHeight;
	}
	static inline float getAngle(int pixelValueX){
		return atan( 2.f * (pixelValueX - (HRES/2)) * tan(HFOV) / HRES );
	}
	static float* getGearVector(int leftHeight, int leftWidth, int leftX, int rightHeight, int rightX){
		float d1 = getDistance(leftHeight);
		float d2 = getDistance(rightHeight);
		float pegDistance = (d1 + d2) / 2;
		float pegViewAngle = getAngle(leftX);
		float leftTargetAngle = getAngle((leftX + leftWidth + rightX)/2);

		float pegX = INCH_TO_ENC * (pegDistance * cos(pegViewAngle) + PEG_OFFSET_X);
		float pegY = INCH_TO_ENC * (pegDistance * sin(pegViewAngle) + PEG_OFFSET_Y);
		float pegAngle = leftTargetAngle + acos( (pow(d2,2) - pow(d1,2) - pow(KNOWN_WIDTH,2)) / (-2*d1*KNOWN_WIDTH) ) + (PI/2);
		//printf("leftAngle %f\tpegAngle %f\n", leftTargetAngle, pegViewAngle);
		//printf("leftPeg(h,d): (%d,%f)\trightPeg(h,d): (%d,%f)\n", leftHeight, d1, rightHeight, d2);
		printf("peg vector: (%f\t%f\t%f)\n", pegX, pegY, pegAngle*rad2deg);
		float vector[3] = {pegX, pegY, pegAngle};
		return vector;
	}

	static int VisionThread()
	{
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(160, 120);
		//camera.SetExposureAuto();
		camera.SetExposureManual(10);
		camera.SetBrightness(65);
		camera.SetFPS(10);
		camera.SetWhiteBalanceAuto();

		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Pipeline", 160, 120);
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
			//imwrite("/home/lvuser/image.jpg", source);
			//output = *grip.GetHslThresholdOutput();
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
			if(contours.size() == 2){
				int leftT  = (r[0].x < r[1].x) ? 0 : 1;
				int rightT = (leftT == 1) ? 0 : 1;
				center 		= r[leftT].x + r[leftT].width + (abs( r[leftT].x + r[leftT].width - r[rightT].x ) >> 1);
				int leftHeight	= r[leftT].height;
				int rightHeight	= r[rightT].height;

				getGearVector(r[leftT].height, r[leftT].width, r[leftT].x, r[rightT].height, r[rightT].x);
				circle( drawing, cv::Point(center, r[leftT].y+(r[leftT].height >> 1)), abs(leftHeight - rightHeight), color3, 1, cv::LINE_4, 0 );
			}
			outputStreamStd.PutFrame(drawing);
			std::this_thread::__sleep_for(std::chrono::seconds(1), std::chrono::milliseconds(0));

		}
		//	return center;
	}

	void RobotInit(void) override
	{
		//CameraServer::GetInstance()->StartAutomaticCapture();

		//variables
		manipState = 0;
		fileCount = 1;
		autoMode = 0;
		turnSide = RED_SIDE;
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

		m_intoShooter = new VictorSP(6);
		m_intake = new VictorSP(4);

		//Talons
		m_shooterB = new CANTalon(1);
		m_shooterB->SetControlMode(CANSpeedController::kPercentVbus); // BEN A (makes deceleration coast)
		m_shooterB->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_shooterB->ConfigEncoderCodesPerRev(4096);
		m_shooterB->SetSensorDirection(true);
		m_shooterB->SelectProfileSlot(0);
		m_shooterB->SetPID(0.05, 0, 0.4, 0.033);
		m_shooterB->SetCloseLoopRampRate(15);
		m_shooterB->SetAllowableClosedLoopErr(0);

		m_shooterA = new CANTalon(3);
		m_shooterA->SetControlMode(CANSpeedController::kFollower);
		m_shooterA->Set(1);

		m_climber = new CANTalon(2);
		m_climber->SetControlMode(CANSpeedController::kPosition);
		//m_climber->SetControlMode(CANSpeedController::kSpeed);
		m_climber->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_climber->ConfigEncoderCodesPerRev(4096);
		m_climber->SetSensorDirection(true); // BEN A
		m_climber->SetPID(1, 0, 0, 0); // BEN A
		m_climber->SetCloseLoopRampRate(0);
		m_climber->SetAllowableClosedLoopErr(0);
		m_climber->SelectProfileSlot(0);
		m_climber->SetPosition(0.0); // BEN A
		lastClimberPos = 0.0; // BEN A

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
//#ifdef PRACTICE_BOT
		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_gearPushOut = new Solenoid(2);
		m_gearPushIn = new Solenoid(3);
		m_introducerIn = new Solenoid(6); //4
		m_introducerOut = new Solenoid(7); //5
		m_gearHoldOut = new Solenoid(4); //6
		m_gearHoldIn = new Solenoid(5); //7

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

		//pathTurnPID = new SimPID(1.0, 0, 0.02, 0, 0.087266); //practice bot
		pathTurnPID = new SimPID(0.75, 0, 0.02, 0, 0.087266);
		pathTurnPID->setContinuousAngle(true);

		//pathDrivePID = new SimPID(0.001, 0, 0.0002, 0, 100); practice bot
		pathDrivePID = new SimPID(0.00085, 0, 0.0002, 0, 100);
		pathDrivePID->setMaxOutput(0.9);

		int zero[2] = {0, 0};
		int end[2] = {-6400, 0};
		path_gearCenterPeg = new PathLine(zero, end, 10);
		int cp6[2] = {-3000,-14500};
		int blueEndLoaderHalf[2] = {-9238, -14500};
		int centerPegBlueEndLoader[2] = {-37511, -14500};
		path_gearCenterPegBlue2 = new PathCurve(end, zero, cp6, blueEndLoaderHalf, 40);
		Path *temp = new PathLine(blueEndLoaderHalf, centerPegBlueEndLoader, 10);
		path_gearCenterPegBlue2->add(temp);
		delete temp;

		int cp1[2] = {-7000, 0};
		int cp2[2] = {-8000, 1000}; //{-9000, 1000};
		int leftPegEnd[2] = {-10100, -2300};//{-10800, -2700};
		int leftShotEnd[2] = {-1000, 5041};//{-270, 5941};
		path_gearLeftPeg = new PathCurve(zero, cp1, cp2, leftPegEnd, 40);
		cp1[0] = -5544;
		cp1[1] = 731;
		cp2[0] = -7749;
		cp2[1] = 1631;
		path_gearLeftPeg2 = new PathCurve(leftPegEnd, cp2, cp1, leftShotEnd, 40); //angle 43

		int cp3[2] = {-7000, 0};
		int cp4[2] = {-6000, -1000};
		int RightPegEnd[2] = {-9500, 4750};//{-9300, 5000};
		int RightLoadEnd[2] = {-37511, -2029};
		path_gearRightPeg = new PathCurve(zero, cp3, cp4, RightPegEnd, 40);
		cp3[0] = -6500;
		cp3[1] = 670;
		cp4[0] = -28700;
		cp4[1] = -1345;
		path_gearRightPeg2 = new PathCurve(RightPegEnd, cp3, cp4, RightLoadEnd, 60);

		CLAMPS = new PathFollower(500, PI/3, pathDrivePID, pathTurnPID);
		CLAMPS->setIsDegrees(true);

		//int cp5[2] = {-3800, 0};
		//int cp6[2] = {0, 7900};
		int RightShot1End[2] = {-3800, 7900};
		int LeftShot1End[2] = {-3800, -7900};
		path_rightShot1 = new PathLine(zero, RightShot1End , 2);
		//path_rightShot2 = new PathCurve(,);
		path_leftShot1 = new PathLine(zero, LeftShot1End, 2);
		//path_leftShot2 = new PathCurve(zero,);
		//CLAMPS = new PathFollower(500, PI/3, pathDrivePID, pathTurnPID);
		//CLAMPS->setIsDegrees(true);
	}

	void DisabledInit()
	{
		m_gearLED->Set(Relay::kOff);
		m_shotLED->Set(Relay::kOff);
		lastClimberPos = m_climber->GetPosition();
		m_climber->Set(lastClimberPos);
	}

	void DisabledPeriodic()
	{
//#ifndef PRACTICE_BOT
		DriverStation::ReportError("Left encoder" + std::to_string((long)m_leftEncoder->Get()) + "Right Encoder" + std::to_string((long)m_rightEncoder->Get()) + "Gyro" + std::to_string(nav->GetYaw()));
		DriverStation::ReportError("Auto Mode: " + std::to_string(autoMode) );
//#endif
		if(m_Joystick->GetRawButton(11)) {
			turnSide = RED_SIDE;
			DriverStation::ReportError("Turn Side: RED");
		}
		else if(m_Joystick->GetRawButton(12)) {
			turnSide = BLUE_SIDE;
			DriverStation::ReportError("Turn Side: BLUE");
		}

		for(int i = 1; i <= 10; i++) {
			if(m_Joystick->GetRawButton(i)){
				autoMode = i;
				nav->ZeroYaw();
				m_leftEncoder->Reset();
				m_rightEncoder->Reset();
				CLAMPS->reset();
				autoState = 0;
			}
		}

		CLAMPS->updatePos(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw());
		printf("robot position x: %d\ty:%d\n", CLAMPS->getXPos(), CLAMPS->getYPos());
	}

	void AutonomousInit()
	{
		autoState = 0;
		nav->ZeroYaw();
		m_leftEncoder->Reset();
		m_rightEncoder->Reset();
		m_shooterB->SetControlMode(CANSpeedController::kPercentVbus); // BEN A (makes deceleration coast)
		m_shooterB->Set(0.f);
		m_gearLED->Set(Relay::kOn);
		autoTimer->Reset();
	}

	void AutonomousPeriodic()
	{
		switch(autoMode) {
		case 0: //still
			m_leftDrive0->SetSpeed(0.f);
			m_leftDrive1->SetSpeed(0.f);
			m_rightDrive2->SetSpeed(0.f);
			m_rightDrive3->SetSpeed(0.f);
			m_intake->SetSpeed(0.f);
			m_shooterB->SetControlMode(CANSpeedController::kPercentVbus); // BEN A (makes deceleration coast)
			m_shooterB->Set(0.f);
			m_intoShooter->SetSpeed(0.f);
			break;
		case 1: //load on middle peg, drive along wall to load station
			switch(autoState)
			{
			case 0:
				m_leftDrive0->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
				m_intake->SetSpeed(0.f);
				m_shooterB->SetControlMode(CANSpeedController::kPercentVbus); // BEN A (makes deceleration coast)
				m_shooterB->Set(0.f);
				m_intoShooter->SetSpeed(0.f);
				CLAMPS->initPath(path_gearCenterPeg, PathBackward, 0);
				autoState++;
				break;
			case 1:
				if (advancedAutoDrive()){
					autoState++;
					agTimer->Reset();
					agTimer->Start();
				}
				break;
			case 2://activate plunger
				m_gearPushIn->Set(false);
				m_gearPushOut->Set(true);
				if(agTimer->Get() > 0.5) {
					autoState++;
					if(turnSide == BLUE_SIDE)
						CLAMPS->initPath(path_gearCenterPegBlue2, PathForward, 0);
				}
				break;
			case 3: //drive away to loader
				if(CLAMPS->getDistance() < 28747){
					//go to high gear, needs different PID setting
					//m_shiftHigh->Set(true);
					//m_shiftLow->Set(false);
				}
				advancedAutoDrive();
			}
			break;

		case 2: //Autonomous mode 2: Load GEAR onto left Gear Peg
			switch(autoState){
			case 0: //Initial case. All motors and actuators are stopped lest a command is carried over from the previous robot session
				m_leftDrive0->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
				m_intake->SetSpeed(0.f);
				m_shooterB->SetControlMode(CANSpeedController::kPercentVbus);
				m_shooterB->Set(0.f);
				m_intoShooter->SetSpeed(0.f);
				CLAMPS->initPath(path_gearLeftPeg, PathBackward, 60);
				/* The initPath protocol is part of ShiftLib's path program
				 * The program uses algorithms from angles and encoder outputs to determine it's position in two dimensions as opposed to one
				 * The first input is an array of coordinates that form a Bezier curve
				 * The second input is the direction the robot must face. In this case, it faces backward.
				 * The third input is the angle at which the robot rests, determined by the gyroscope
				 * This is phase 1, where the path has been determined
				 */
				autoState++;
				break;
			case 1: //First case. Path program is in phase two, wherein the robot follows the predetermined path. Autonomous mode ends.
				if (advancedAutoDrive()){
					autoState ++;
					agTimer->Reset();
					agTimer->Start();
				}
				break;
			case 2:

				m_gearPushIn->Set(false);
				m_gearPushOut->Set(true);
				advancedAutoDrive();
				if(agTimer->Get() > 0.5) {
					if(turnSide == BLUE_SIDE){
						autoState++;
						CLAMPS->initPath(path_gearLeftPeg2, PathForward, 43);
					}
					else{
						autoState++;
						CLAMPS->initPath(path_gearLeftPeg2, PathForward, 43);
					}
				}
				break;
			case 3:
				setPoint = SHOOTER_SPEED;
				m_shooterB->SetControlMode(CANSpeedController::kSpeed); // BEN A (makes deceleration coast)
				m_shooterB->Set(setPoint);
				if(advancedAutoDrive())
					autoState++;
				break;
			case 4:
				if(fabs(m_shooterB->GetSpeed() - setPoint) < 0.04 * fabs(setPoint)) //practice 0.06
					m_intoShooter->SetSpeed(0.8);
				else
					m_intoShooter->SetSpeed(0.f);

				if (agTimer->Get() > 8){
					autoState++;
				}
				break;
			case 5:
				m_shooterB->Set(0.0f);
				m_intoShooter->SetSpeed(0.f);
				break;
			}
			break;

		case 3: //right peg auto
			switch(autoState){
			case 0:
				m_leftDrive0->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
				m_intake->SetSpeed(0.f);
				m_shooterB->SetControlMode(CANSpeedController::kPercentVbus);
				m_shooterB->Set(0.f);
				m_intoShooter->SetSpeed(0.f);
				CLAMPS->initPath(path_gearRightPeg, PathBackward, -60);

				autoState ++;
				break;
			case 1:
				if(advancedAutoDrive()){
					autoState++;
					agTimer->Reset();
					agTimer->Start();
				}
				break;
			case 2: //plunge
				m_gearPushIn->Set(false);
				m_gearPushOut->Set(true);
				advancedAutoDrive();
				if(agTimer->Get() > 0.5) {
					autoState++;
					//drive to loader station
					CLAMPS->initPath(path_gearRightPeg2, PathForward, 0);
				}
				break;
			case 3: //drive away to loader
				advancedAutoDrive();
			}

			break;

		case 4:
			switch(autoState) {
			case 0:
				m_leftDrive0->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
				m_intake->SetSpeed(0.f);
				m_shooterB->SetControlMode(CANSpeedController::kPercentVbus);
				m_shooterB->Set(0.f);
				m_intoShooter->SetSpeed(0.f);
				//blue side
				if(turnSide == BLUE_SIDE)
					CLAMPS->initPath(path_rightShot1, PathBackward, -7.5);
				else if (turnSide == RED_SIDE)
					CLAMPS->initPath(path_leftShot1, PathBackward, 7.5);
				agTimer->Start();
				//agLastTime = agTimer->Get();
				autoState ++;
				break;
			case 1: //shoot balls for 8 seconds
				setPoint = AUTO_SHOOTER_SPEED;
				m_shooterB->SetControlMode(CANSpeedController::kSpeed); // BEN A (makes deceleration coast)
				m_shooterB->Set(setPoint);
				if(fabs(m_shooterB->GetSpeed() - setPoint) < 0.06 * fabs(setPoint)) // BEAN (Old conditional wasn't working)
					m_intoShooter->SetSpeed(1.0);
				else
					m_intoShooter->SetSpeed(0.f);

				if (agTimer->Get() > 8){
					autoState++;
				}
				break;
			case 2:
				m_shooterB->Set(0.0f);
				m_intoShooter->SetSpeed(0.f);
				advancedAutoDrive();
				break;
			}
		break;
		}
	}

	void TeleopInit() {
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		m_gearLED->Set(Relay::kOn);
		lastClimberPos = m_climber->GetPosition();
		m_climber->Set(lastClimberPos);
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
		if(m_Gamepad->GetTriggerAxis(GenericHID::JoystickHand::kLeftHand) > 0.9) {
			setPoint = -3225;
			m_introducerOut->Set(false);
			m_introducerIn->Set(true);
		}
		else {
			m_introducerOut->Set(true);
			m_introducerIn->Set(false);
			setPoint = -3210;
		}
	}*/

	void trim(){
		float motorOutput = 0.5*m_Joystick->GetRawAxis(3) + 0.5;

		m_shooterB->SetControlMode(CANSpeedController::kPercentVbus); // BEN A (makes deceleration coast)
		m_shooterB->Set(motorOutput);
		float encoderRPM = m_shooterB->GetSpeed()*18.75f/128.f;

		DriverStation::ReportError("Encoder speed" + std::to_string((long)encoderRPM));


	}

	void ShooterPID() {

		//int setPoint = -(1500 * (0.5 * m_Joystick->GetRawAxis(3) + 0.5) + 2000);
		if(m_Gamepad->GetTriggerAxis(GenericHID::JoystickHand::kLeftHand) > 0.9) {
		/*	if(agTimer->Get() > 3.0)
				m_intoShooter->SetSpeed(1.0);
			if(agTimer->Get() > 6.0) {
	*/
				setPoint = -3225;
				m_introducerOut->Set(false);
				m_introducerIn->Set(true);
			}
			else {
				m_introducerOut->Set(true);
				m_introducerIn->Set(false);
				setPoint = SHOOTER_SPEED;
			}

		gettimeofday(&tv, 0);

		float encoderRPM = m_shooterB->GetSpeed();
		DriverStation::ReportError("setpoint: "+ std::to_string(setPoint) + "Encoder speed" + std::to_string((long)encoderRPM));

		if(m_Gamepad->GetAButton()) {
			agTimer->Start();
			m_shooterB->SetControlMode(CANSpeedController::kSpeed); // BEN A (makes deceleration coast)
			m_shooterB->Set(setPoint);
			//->Set(SHOOTER_RATIO);

			if(fabs(m_shooterB->GetSpeed() - setPoint) < 0.04 * fabs(setPoint)) // practice bot was 0.6
				m_intoShooter->SetSpeed(0.8); //practice bot is 1.0
			else
				m_intoShooter->SetSpeed(0.f);


			m_shooterB->Set(setPoint);
			DriverStation::ReportError("speed error " + std::to_string(m_shooterB->GetClosedLoopError()*NATIVE_TO_RPM));

			sprintf(buffer, "%d:%d , %d , %d , %f\n", (int)tv.tv_sec, (int)tv.tv_usec, setPoint, (int)encoderRPM, m_shooterB->GetClosedLoopError()*NATIVE_TO_RPM);
			file << buffer;

			fileCount++;
		}
		else if(m_Gamepad->GetStartButton()) {
			m_shooterB->SetControlMode(CANSpeedController::kSpeed); // BEN A (makes deceleration coast)
			m_shooterB->Set(0.6 * SHOOTER_RPM);
			m_intoShooter->SetSpeed(-0.2);
		}
		else if(m_Gamepad->GetBackButton()) {
			m_shooterB->SetControlMode(CANSpeedController::kSpeed); // BEN A (makes deceleration coast)
			m_shooterB->Set(setPoint);
		}
		else if(m_Gamepad->GetBButton()) {
			m_intoShooter->SetSpeed(0.5);
		}
			else
		{
			m_shooterB->SetControlMode(CANSpeedController::kPercentVbus); // BEN A (makes deceleration coast)
			m_shooterB->Set(0);
			//->Set(0.f);
			m_intoShooter->SetSpeed(0.f);
			//m_introducerOut->Set(true);
			//m_introducerIn->Set(false);
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

		if(m_Gamepad->GetXButton()) {
			m_gearPushIn->Set(false);
			m_gearPushOut->Set(true);
		}
		else if(m_Gamepad->GetYButton()){
			m_gearPushIn->Set(true);
			m_gearPushOut->Set(false);
		}
		if(m_Gamepad->GetTriggerAxis(GenericHID::JoystickHand::kRightHand) < 0.9 ) {
			m_gearHoldOut->Set(false);
			m_gearHoldIn->Set(true);
		}
		else {
			m_gearHoldOut->Set(true);
			m_gearHoldIn->Set(false);
				}
	}

	void advancedClimb() {
		double target; // BEN A made many changes here
		if(m_Gamepad->GetPOV(0) == DP_UP)
			target = m_climber->GetPosition() - 1/m_climber->GetP();
		else if(m_Gamepad->GetPOV(0) == DP_DOWN)
			target = m_climber->GetPosition() + 1/m_climber->GetP();
		else
			target = lastClimberPos;

		m_climber->Set(target);
		lastClimberPos = target;

		DriverStation::ReportError("ClimberPos" + std::to_string((long)m_climber->GetPosition()));
	}

	/*void simpleClimb() {
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
		DriverStation::ReportError("ClimberPos" + std::to_string((long)m_climber->GetPosition()));
	}*/

//=====================VISION FUNCTIONS=====================


	/*bool aimAtTarget() {
		float turn = last_turn;

		return 0;
	}*/

	/*bool lineUpGear() {//travis did this :3
		int targetCenter;
		return false;
	}*/

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

	bool advancedAutoDrive(){
		if(CLAMPS->followPathByEnc(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw(), leftSpeed, rightSpeed) == 0){
			m_leftDrive0->SetSpeed(leftSpeed);
			m_leftDrive1->SetSpeed(leftSpeed);
			m_rightDrive2->SetSpeed(rightSpeed);
			m_rightDrive3->SetSpeed(rightSpeed);
		}
		printf("path follow left: %f, right: %f\n", leftSpeed, rightSpeed);
		return CLAMPS->isDone();
	}

//=======================MATHY FUNCTIONS============================
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
