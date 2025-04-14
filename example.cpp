#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANdi.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "units/current.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include "units/angular_acceleration.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/frequency.h"
#include "units/time.h"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/ColorFlowAnimation.h"
std::string interface = "can0";

enum LEDState
{
	Accelerating,
	Stopping,
	ShowScore
};

int main() {
	/* Setup configs */
	ctre::phoenix6::hardware::TalonFX m_motor{0, interface};
	ctre::phoenix6::hardware::CANdi m_candi{0, interface};
	ctre::phoenix6::configs::TalonFXConfiguration m_motorConfigs{};
	ctre::phoenix::led::CANdle m_candle{0, interface};

	m_motorConfigs.CurrentLimits.StatorCurrentLimit = 10_A;
	m_motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

	m_motorConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 10_A;
	m_motorConfigs.TorqueCurrent.PeakReverseTorqueCurrent = 10_A;

	m_motorConfigs.Slot0.kP = 3;
	m_motorConfigs.Slot0.kI = 0;
	m_motorConfigs.Slot0.kD = 0;
	m_motorConfigs.Slot0.kS = 2.15;
	m_motorConfigs.Slot0.kV = 0.02;
	m_motorConfigs.Slot0.kA = 0;
	m_motorConfigs.Slot0.kG = 0;

	auto& configurator = m_motor.GetConfigurator();

	for (int i = 0; i <= 5; ++i) {
		auto configErr = configurator.Apply(m_motorConfigs);

		/* Break early because configs succeeded */
		if (configErr.IsOK()) {
			break;
		}
	}

	/* Setup controls */
	ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC m_control{5_tps};

	/* Setup status signals */
	auto& magEncoderPosition = m_candi.GetPWM1Position();
	auto& buttonClosedState = m_candi.GetS2Closed();

	ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
		500_Hz,
		magEncoderPosition,
		buttonClosedState
	);

	bool shouldAccelerate = true;

	ctre::phoenix::led::ColorFlowAnimation m_ledAnimation{0, 255, 0, 0};
	LEDState ledState = LEDState::Accelerating;

	/* Int timer index for state machine logic */
	int timer = 0;

	/* How fast the main loop should run at */
	int loopFrequencyHz = 250;

	while (true) {
		ctre::phoenix6::BaseStatusSignal::WaitForAll(
			10_ms,
			magEncoderPosition,
			buttonClosedState
		);

		/* Button is pressed and we are currently accelerating */
		if (buttonClosedState.GetValue() == true && shouldAccelerate) {
			shouldAccelerate = false; /* stop the mechanism */
			ledState = LEDState::Stopping;
		} else if (buttonClosedState.GetValue() == false && !shouldAccelerate) {
			shouldAccelerate = true; /* start the mechanism and reset LEDs*/
			ledState = LEDState::Accelerating;
		}

		/* Small state machine to handle showing score and LED animations */
		switch (ledState) {
			case LEDState::Accelerating:
				/* Disable the LEDs*/
				m_candle.SetLEDs(0, 0, 0, 0, 0, 7);

				/* Reset timer */
				timer = 0;
				break;
			case LEDState::Stopping:
				if (timer > loopFrequencyHz * 10) {
					/* Reset timer */
					timer = 0;

					/* Show score after 2 seconds */
					ledState = LEDState::ShowScore;
					break;
				} 
				
				m_candle.Animate(m_ledAnimation);
				timer++;
				break;
			case LEDState::ShowScore:
				double positionDegrees = fmod(magEncoderPosition.GetValueAsDouble(), 1) * 360;
				int score = 0;
				if (positionDegrees < 45) {
					score = 4;
				} else if (positionDegrees < 90) {
					score = 3;
				} else if (positionDegrees < 170) {
					score = 2;
				} else if (positionDegrees < 180) {
					score = 8;
				} else if (positionDegrees < 225) {
					score = 6;
				} else if (positionDegrees < 270) {
					score = 6;
				} else if (positionDegrees < 330) {
					score = 2;
				} else {
					score = 7;
				}

				m_candle.SetLEDs(0, 255, 0, 0, 0, score);
				break;
		}

		if (shouldAccelerate) {
			/* we should be accelerating, so set control with fast acceleration */
			m_motor.SetControl(m_control
				.WithUpdateFreqHz(0_Hz)
				.WithVelocity(5_tps)
				.WithAcceleration(10_tr_per_s_sq));
		} else {
			/* we should not be accelerating, so set velocity to zero with a very slow acceleration */
			m_motor.SetControl(m_control
				.WithUpdateFreqHz(0_Hz)
				.WithVelocity(0_tps)
				.WithAcceleration(0.25_tr_per_s_sq));
		}

		/* Timeout if we haven't received a control signal in 20ms */
		/* Feed enable is required for actuators/motors to enable */
		ctre::phoenix::unmanaged::FeedEnable(20);

		/* 5ms control loop */
		std::this_thread::sleep_for(std::chrono::milliseconds(1000/loopFrequencyHz));
	}

	return 0;
}
