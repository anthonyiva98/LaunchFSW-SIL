#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>

#include "TestHelpers.h"

#include "FlightTypes.h"
#include "sim/BasicVehicleModel.h"

#include <chrono>

using namespace std::chrono_literals;
using namespace Simulation;
using namespace FlightCore;

namespace
{
	enum class NominalVehicleStage
	{
		INITIAL,
		ENGINE_RUNNING,
		LIFTOFF_DETECTED,
		CUTOFF_CONDITION_MET,
		ENGINE_STOPPED
	};

	struct BasicVehicleTestScenario
	{
		BasicVehicleModelConfig m_config;
		BasicVehicleModel m_vehicle;

		SensorSnapshot m_snapshot;
		Duration m_testElapsed = Duration::zero();
		NominalVehicleStage m_stage = NominalVehicleStage::INITIAL;

		BasicVehicleTestScenario(BasicVehicleModelConfig in_config = TestHelpers::MakeBasicVehicleConfig()) : m_config(in_config), m_vehicle(m_config)
		{
		}

		void Step(bool ignitionCommand, Duration deltaTime)
		{
			m_testElapsed += deltaTime;
			m_snapshot = m_vehicle.Step(ignitionCommand, deltaTime);
		}

		void DriveNominallyTo(NominalVehicleStage targetStage)
		{
			REQUIRE( static_cast<int>(targetStage) >= static_cast<int>(m_stage));

			while (m_stage != targetStage)
			{
				switch (m_stage)
				{
				case NominalVehicleStage::INITIAL:
					Step(true, m_config.m_engineStartDelay);

					REQUIRE(m_snapshot.m_bEngineRunning);
					REQUIRE_FALSE(m_snapshot.m_bLiftoffDetected);
					REQUIRE_FALSE(m_snapshot.m_bCutoffConditionMet);

					m_stage = NominalVehicleStage::ENGINE_RUNNING;
					break;

				case NominalVehicleStage::ENGINE_RUNNING:
					Step(true, m_config.m_liftoffDelay);

					REQUIRE(m_snapshot.m_bEngineRunning);
					REQUIRE(m_snapshot.m_bLiftoffDetected);
					REQUIRE_FALSE(m_snapshot.m_bCutoffConditionMet);

					m_stage = NominalVehicleStage::LIFTOFF_DETECTED;
					break;

				case NominalVehicleStage::LIFTOFF_DETECTED:
					Step(true, m_config.m_poweredAscentDuration);

					REQUIRE(m_snapshot.m_bEngineRunning);
					REQUIRE(m_snapshot.m_bLiftoffDetected);
					REQUIRE(m_snapshot.m_bCutoffConditionMet);

					m_stage =
						NominalVehicleStage::CUTOFF_CONDITION_MET;
					break;

				case NominalVehicleStage::CUTOFF_CONDITION_MET:
					Step(false, m_config.m_engineStopDelay);

					REQUIRE_FALSE(m_snapshot.m_bEngineRunning);
					REQUIRE(m_snapshot.m_bLiftoffDetected);
					REQUIRE(m_snapshot.m_bCutoffConditionMet);

					m_stage = NominalVehicleStage::ENGINE_STOPPED;
					break;

				case NominalVehicleStage::ENGINE_STOPPED:
					FAIL("Cannot advance beyond nominal engine shutdown");
				}
			}
		}
	};

}


TEST_CASE("BasicVehicleModel follows nominal vehicle sequence", "[BasicVehicleModel]")
{
	BasicVehicleTestScenario scenario;

	constexpr auto tickDuration = 1s;

	const Duration ignitionStartTime = scenario.m_testElapsed;
	scenario.Step(true, tickDuration);

	REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);

	// Active timer reaches its deadline
	const Duration engineStartTargetTime = ignitionStartTime + scenario.m_config.m_engineStartDelay;
	scenario.Step(true, engineStartTargetTime - scenario.m_testElapsed);

	{
		INFO("Expected engine start at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// The engine transition started the liftoff timer
	const Duration liftoffStartTime = scenario.m_testElapsed;
	const Duration liftoffTargetTime = liftoffStartTime + scenario.m_config.m_liftoffDelay;
	scenario.Step(true, liftoffTargetTime - scenario.m_testElapsed);

	{
		INFO("expected liftoff at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// Liftoff started the powered-flight timer
	const Duration poweredAscentStartTime = scenario.m_testElapsed;
	const Duration poweredAscentTargetTime = poweredAscentStartTime + scenario.m_config.m_poweredAscentDuration;
	scenario.Step(true, poweredAscentTargetTime - scenario.m_testElapsed);

	{
		INFO("Expected cutoff conditions met at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// Falling edge starts engine-stop timer
	const Duration shutdownStartTime = scenario.m_testElapsed;
	scenario.Step(false, tickDuration);
	{
		INFO("Expected engine still running at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// Active stop timer reaches its deadline
	const Duration shutdownTargetTime = shutdownStartTime + scenario.m_config.m_engineStopDelay;
	scenario.Step(false, shutdownTargetTime - scenario.m_testElapsed);

	{
		INFO("Expected engine stopped at time: " << scenario.m_testElapsed.count());

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

}


TEST_CASE("BasicVehicleModel applies inhibit events", "[BasicVehicleModel]")
{
	BasicVehicleTestScenario scenario;

	SECTION("Engine start is inhibited") 
	{
		scenario.DriveNominallyTo(NominalVehicleStage::INITIAL);
		scenario.m_vehicle.ApplyEvent(VehicleEvent::INHIBIT_ENGINE_START);

		scenario.Step(true, scenario.m_config.m_engineStartDelay);

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);

	}

	SECTION("Liftoff is inhibited") 
	{
		scenario.DriveNominallyTo( NominalVehicleStage::ENGINE_RUNNING);
		scenario.m_vehicle.ApplyEvent( VehicleEvent::INHIBIT_LIFTOFF);

		scenario.Step( true, scenario.m_config.m_liftoffDelay);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}
	
	SECTION("Engine stop is inhibited") 
	{
		scenario.DriveNominallyTo(NominalVehicleStage::CUTOFF_CONDITION_MET);
		scenario.m_vehicle.ApplyEvent(VehicleEvent::INHIBIT_ENGINE_STOP);

		scenario.Step(false, scenario.m_config.m_engineStopDelay);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	
	}

	SECTION("Inhibit events remain latched")
	{
		scenario.DriveNominallyTo(NominalVehicleStage::INITIAL);

		scenario.m_vehicle.ApplyEvent(VehicleEvent::INHIBIT_ENGINE_START);
		scenario.Step(true, scenario.m_config.m_engineStartDelay);

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);

		// Create a second genuine ignition edge.
		scenario.m_vehicle.ApplyEvent(VehicleEvent::NONE);
		scenario.Step(false, 1s);
		scenario.Step(true, scenario.m_config.m_engineStartDelay);

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	SECTION("Late inhibit does not reverse an existing observation")
	{
		scenario.DriveNominallyTo(NominalVehicleStage::LIFTOFF_DETECTED);
		scenario.m_vehicle.ApplyEvent(VehicleEvent::INHIBIT_LIFTOFF);

		scenario.Step(true, 1s);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);

	}

}


TEST_CASE("BasicVehicleModel applies forced events", "[BasicVehicleModel]")
{
	BasicVehicleTestScenario scenario;

	SECTION("Forced engine stop")
	{
		scenario.DriveNominallyTo(NominalVehicleStage::LIFTOFF_DETECTED);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);

		scenario.m_vehicle.ApplyEvent(VehicleEvent::FORCE_ENGINE_STOP);

		scenario.Step(true, 1s);

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	SECTION("Forced engine restart")
	{

		scenario.DriveNominallyTo(NominalVehicleStage::ENGINE_STOPPED);

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);

		scenario.m_vehicle.ApplyEvent(VehicleEvent::FORCE_ENGINE_RESTART);
		scenario.Step(false, 1s);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	SECTION("Forced restart cancels an active stop timer")
	{
		scenario.DriveNominallyTo(NominalVehicleStage::CUTOFF_CONDITION_MET);

		scenario.Step(false, 1s);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);

		scenario.m_vehicle.ApplyEvent(VehicleEvent::FORCE_ENGINE_RESTART);
		scenario.Step(false, 1s);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);

		scenario.Step(false, scenario.m_config.m_engineStopDelay);

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);

	}
}


TEST_CASE("BasicVehicleModel rounds transitions to the next tick boundary", "[BasicVehicleModel]")
{
	BasicVehicleTestScenario scenario( TestHelpers::MakeBasicVehicleConfig(2500ms));

	scenario.Step(true, 1s);
	REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);

	scenario.Step(true, 1s);
	REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);

	scenario.Step(true, 1s);
	REQUIRE(scenario.m_snapshot.m_bEngineRunning);
}


TEST_CASE("BasicVehicleModel follows nominal vehicle sequence when no delays", "[BasicVehicleModel]")
{
	BasicVehicleTestScenario scenario(TestHelpers::MakeBasicVehicleConfig(0s, 0s, 0s, 0s));

	// Input edge begins engine-start timer immediately.
	scenario.Step(true, 1s);

	{
		INFO("Expected engine start at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// Liftoff timer was created by the preceding transition.
	scenario.Step(true, 1s);

	{
		INFO("expected liftoff at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// Powered-flight timer was created by the preceding transition.
	scenario.Step(true, 1s);

	{
		INFO("Expected cutoff conditions met at time: " << scenario.m_testElapsed.count());

		REQUIRE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

	// Falling input edge begins the zero-delay stop timer immediately.
	scenario.Step(false, 1s);
	{
		INFO( "Expected engine stopped at time: " << scenario.m_testElapsed.count());

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE(scenario.m_snapshot.m_bCutoffConditionMet);
	}

}


TEST_CASE("BasicVehicleModel Pending engine start is cancelled", "[BasicVehicleModel]")
{
	constexpr auto tickDuration = 1s;
	BasicVehicleTestScenario scenario;

	// Ignition cmd true, engine-start delay not hit -> engine_running = false
	scenario.Step(true, tickDuration);
	REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);

	// Ignition cmd false, engine-start delay not hit -> engine_running = false
	scenario.Step(false, tickDuration);
	REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);

	// Ignition cmd false, engine-start deadline passed -> engine_running = false
	scenario.Step(false, scenario.m_config.m_engineStartDelay);
	REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);

}


TEST_CASE( "BasicVehicleModel remains idle without ignition", "[BasicVehicleModel]")
{
	BasicVehicleTestScenario scenario;
	constexpr auto tickDuration = 1s;

	const Duration fullTestDelay = 
		scenario.m_config.m_engineStartDelay + 
		scenario.m_config.m_liftoffDelay + 
		scenario.m_config.m_poweredAscentDuration + 
		scenario.m_config.m_engineStopDelay + 
		1s;

	int tick = 0;

	while (scenario.m_testElapsed < fullTestDelay)
	{
		scenario.Step(false, tickDuration);

		INFO("Unexpected vehicle activity on tick " << tick);

		REQUIRE_FALSE(scenario.m_snapshot.m_bEngineRunning);
		REQUIRE_FALSE(scenario.m_snapshot.m_bLiftoffDetected);
		REQUIRE_FALSE(scenario.m_snapshot.m_bCutoffConditionMet);
		++tick;

	}


}

