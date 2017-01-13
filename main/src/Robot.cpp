#include <iostream>
#include <memory>
#include <string>
#include "WPILib.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

class Robot: public frc::IterativeRobot {
public:
	Victor *m_leftDrive0; //4
	Victor *m_leftDrive1; //1
	Victor *m_rightDrive2; //2
	Victor *m_rightDrive3; //3
	VictorSP *m_thing;
	Joystick *m_Joystick;
	XboxController *m_Gamepad;
	void RobotInit() {
		m_leftDrive0 = new Victor(0);
		m_leftDrive1 = new Victor(1);
		m_rightDrive2 = new Victor(2);
		m_rightDrive3 = new Victor(3);
		m_thing = new VictorSP(4);
		m_Gamepad = new XboxController(0);
	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		operateThing();
	}

	void TestPeriodic() {

	}

	void operateThing() {
		if (m_Gamepad->GetYButton())
			m_thing->SetSpeed(1.0);
		else
			m_thing->SetSpeed(0.0);
	}

	void teleDrive() {
/*		leftSpeed = m_Joystick->GetY() - m_Joystick->GetX()
		rightSpeed = 1) - scale(limit(expo(m_Gamepad2->GetRawAxis(4), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) + scale(-limit(expo(m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) + scale(expo(-m_Gamepad->GetRawAxis(1), 2), 0.5) - scale(expo(m_Gamepad->GetRawAxis(0), 3), 0.5);

		m_leftDrive0->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);*/
	}

};

START_ROBOT_CLASS(Robot)
