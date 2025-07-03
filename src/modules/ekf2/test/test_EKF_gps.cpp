#include <gtest/gtest.h>
#include <iostream>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

using namespace std;

class EkfGpsTest : public ::testing::Test
{
public:
	EkfGpsTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EkfGpsTest, VerticalTakeoffLanding)
{
    EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
    std::ofstream log_file("gps_test_vertical.csv");
    log_file << "timestamp,baro_pressure,gps_alt,gps_fix_type,gps_num_sats,imu_accel_z,ekf_pos_z,ekf_vel_z\n";
    _ekf->set_in_air_status(false);
    _ekf->set_vehicle_at_rest(true);
    uint64_t start_time = _sensor_simulator.getTime();


    float climb_speed = 2.0f;
    const float climb_duration = 5.0f;
    float current_altitude = 0.0f;
    const float sea_level_pressure = 101.325;

    log_file << "# === TAKEOFF PHASE ===\n";
    for (float t = 0; t < climb_duration; t += 0.1f) {
        current_altitude += climb_speed * 0.1f;

        float baro_pressure = sea_level_pressure * (1 - _sensor_simulator._gps.getData().alt / 8000.0f);
        _sensor_simulator._baro.setData(baro_pressure);

        _sensor_simulator._gps.setAltitude(_sensor_simulator._gps.getDefaultGpsData().alt + current_altitude);
        _sensor_simulator._gps.setVelocity(Vector3f(0, 0, -climb_speed)); // NED下速度向上为负

        float upward_accel = 1;
        _sensor_simulator._imu.setData(
            Vector3f(0, 0, -CONSTANTS_ONE_G - upward_accel),
            Vector3f(0, 0, 0)
        );
        climb_speed += upward_accel * 0.1f;
        _sensor_simulator.runSeconds(0.1f);

        float baro_pressure_current = _sensor_simulator._baro.getData();
        float gps_alt_current = _sensor_simulator._gps.getData().alt; // 获取GPS高度
        int gps_fix_type = _sensor_simulator._gps.getData().fix_type;
        int gps_num_sats = _sensor_simulator._gps.getData().nsats;

        Vector3f ekf_position = _ekf->getPosition();
        Vector3f ekf_velocity = _ekf->getVelocity();


        log_file << _sensor_simulator.getTime() - start_time << ","
                 << baro_pressure_current << ","
                 << gps_alt_current << ","
                 << gps_fix_type << ","
                 << gps_num_sats << ","
                 << CONSTANTS_ONE_G - upward_accel << ","
                 << -(ekf_position(2)) + _sensor_simulator._gps.getDefaultGpsData().alt << ","
                 << ekf_velocity(2) << "\n";
    }

    const float hover_duration = 10.0f;

    log_file << "# === HOVER PHASE ===\n";
    for (float t = 0; t < hover_duration; t += 0.1f) {
        _sensor_simulator._imu.setData(
            Vector3f(0, 0, -CONSTANTS_ONE_G),
            Vector3f(0, 0, 0)
        );

        _sensor_simulator._gps.setVelocity(Vector3f(0, 0, 0));

        _sensor_simulator.runSeconds(0.1f);

        float baro_pressure_current = _sensor_simulator._baro.getData();
        float gps_alt_current = _sensor_simulator._gps.getData().alt;

        Vector3f ekf_position = _ekf->getPosition();
        Vector3f ekf_velocity = _ekf->getVelocity();

        log_file << _sensor_simulator.getTime() - start_time << ","
                 << baro_pressure_current << ","
                 << gps_alt_current << ","
                 << _sensor_simulator._gps.getData().fix_type << ","
                 << _sensor_simulator._gps.getData().nsats << ","
                 << CONSTANTS_ONE_G << ","
                 << -ekf_position(2) + _sensor_simulator._gps.getDefaultGpsData().alt << ","
                 << ekf_velocity(2) << "\n";
    }

    const float descend_speed = 1.0f;
    const float descend_duration = 10.0f;

    log_file << "# === LANDING PHASE ===\n";
    for (float t = 0; t < descend_duration; t += 0.1f) {
        current_altitude -= descend_speed * 0.1f;
        current_altitude = std::max(current_altitude, 0.0f);

        float baro_pressure = sea_level_pressure * (1 - _sensor_simulator._gps.getData().alt / 8000.0f);
        _sensor_simulator._baro.setData(baro_pressure);
        _sensor_simulator._gps.setAltitude(_sensor_simulator._gps.getDefaultGpsData().alt + current_altitude);
        _sensor_simulator._gps.setVelocity(Vector3f(0, 0, descend_speed)); // NED下速度向下为正

        const float downward_accel = descend_speed * 0.5f;
        _sensor_simulator._imu.setData(
            Vector3f(0, 0, -CONSTANTS_ONE_G + downward_accel),
            Vector3f(0, 0, 0)
        );

        _sensor_simulator.runSeconds(0.1f);

        float baro_pressure_current = _sensor_simulator._baro.getData();
        float gps_alt_current = _sensor_simulator._gps.getData().alt;

        Vector3f ekf_position = _ekf->getPosition();
        Vector3f ekf_velocity = _ekf->getVelocity();

        log_file << _sensor_simulator.getTime() - start_time << ","
                 << baro_pressure_current << ","
                 << gps_alt_current << ","
                 << _sensor_simulator._gps.getData().fix_type << ","
                 << _sensor_simulator._gps.getData().nsats << ","
                 << CONSTANTS_ONE_G + downward_accel << ","
                 << -ekf_position(2) + _sensor_simulator._gps.getDefaultGpsData().alt << ","
                 << ekf_velocity(2) << "\n";
    }

    log_file << "# === GROUND DETECTION PHASE ===\n";
    _sensor_simulator._imu.setData(
        Vector3f(0, 0, -CONSTANTS_ONE_G),
        Vector3f(0, 0, 0)
    );
    _sensor_simulator._gps.setVelocity(Vector3f(0, 0, 0));

    const float ground_duration = 5.0f;
    for (float t = 0; t < ground_duration; t += 0.1f) {
        _sensor_simulator.runSeconds(0.1f);

        log_file << _sensor_simulator.getTime() - start_time << ","
                 << _sensor_simulator._baro.getData() << ","
                 << _sensor_simulator._gps.getData().alt << ","
                 << _sensor_simulator._gps.getData().fix_type << ","
                 << _sensor_simulator._gps.getData().nsats << ","
                 << CONSTANTS_ONE_G << ","
                 << -_ekf->getPosition()(2) +_sensor_simulator._gps.getDefaultGpsData().alt << ","
                 << _ekf->getVelocity()(2) << "\n";
    }

    //EXPECT_FALSE(_ekf->control_status().flags.in_air);
    //EXPECT_TRUE(_ekf->control_status().flags.vehicle_at_rest);

    log_file.close();

    std::ofstream info_file("gps_test_info.txt");
    info_file << "Vertical Takeoff/Landing Test Data\n";
    info_file << "==================================\n";
    info_file << "Coordinate System: NED (North-East-Down)\n";
    info_file << "  - Z-axis points downward\n";
    info_file << "  - Positive Z velocity = moving down\n";
    info_file << "  - Positive Z position = below origin\n";
    info_file << "Start Time: " << start_time << " μs\n";
    info_file << "End Time: " << _sensor_simulator.getTime() << " μs\n";
    info_file << "Duration: " << (climb_duration + hover_duration + descend_duration + ground_duration) << " s\n";
    info_file << "Max Altitude: " << (climb_speed * climb_duration) << " m (above origin)\n";
    info_file << "Sea Level Pressure Reference: " << sea_level_pressure << " Pa\n";
    info_file << "Gravity Constant: " << CONSTANTS_ONE_G << " m/s²\n";
    info_file.close();
}

TEST_F(EkfGpsTest, gpsTimeout)
{
	// GIVEN:EKF that fuses GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: the number of satellites drops below the minimum
	_sensor_simulator._gps.setNumberOfSatellites(3);

	// THEN: the GNSS fusion does not stop because other metrics are good enough
	_sensor_simulator.runSeconds(8);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: the fix type drops
	_sensor_simulator._gps.setFixType(0);

	// THEN: the GNSS fusion stops after some time
	_sensor_simulator.runSeconds(8);
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());

	// BUT WHEN: the fix type is good again
	_sensor_simulator._gps.setFixType(3);

	// THEN: the GNSS fusion restarts
	_sensor_simulator.runSeconds(6);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
}

TEST_F(EkfGpsTest, gpsFixLoss)
{
	// GIVEN:EKF that fuses GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: the fix is loss
	_sensor_simulator._gps.setFixType(0);

	// THEN: after dead-reconing for a couple of seconds, the local position gets invalidated
	_sensor_simulator.runSeconds(6);
	EXPECT_TRUE(_ekf->control_status_flags().inertial_dead_reckoning);
	EXPECT_FALSE(_ekf->isLocalHorizontalPositionValid());

	// The control logic takes a bit more time to deactivate the GNSS fusion completely
	_sensor_simulator.runSeconds(5);
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
}

TEST_F(EkfGpsTest, resetToGpsVelocity)
{
	ResetLoggingChecker reset_logging_checker(_ekf);
	// GIVEN:EKF that fuses GPS
	// and has gps checks already passed

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	reset_logging_checker.capturePreResetState();

	// AND: simulate constant velocity gps samples for short time
	_sensor_simulator.startGps();
	const Vector3f simulated_velocity(0.5f, 1.0f, -0.3f);
	_sensor_simulator._gps.setVelocity(simulated_velocity);
	const uint64_t dt_us = 1e5;
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f(simulated_velocity) * dt_us * 1e-6);
	_sensor_simulator._gps.stepHeightByMeters(simulated_velocity(2) * dt_us * 1e-6f);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(1.2); // required to pass the checks
	_sensor_simulator.runMicroseconds(dt_us);

	// THEN: a reset to GPS velocity should be done
	const Vector3f estimated_velocity = _ekf->getVelocity();
	EXPECT_NEAR(estimated_velocity(0), simulated_velocity(0), 1e-3f);
	EXPECT_NEAR(estimated_velocity(1), simulated_velocity(1), 1e-3f);
	EXPECT_NEAR(estimated_velocity(2), simulated_velocity(2), 1e-3f);

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(1e-2f));
}

TEST_F(EkfGpsTest, resetToGpsPosition)
{
	// GIVEN:EKF that fuses GPS
	// and has gps checks already passed
	const Vector3f previous_position = _ekf->getPosition();

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// AND: simulate jump in position
	_sensor_simulator.startGps();
	const Vector3f simulated_position_change(20.0f, -1.0f, 0.f);
	_sensor_simulator._gps.stepHorizontalPositionByMeters(
		Vector2f(simulated_position_change));
	_sensor_simulator.runSeconds(6);

	// THEN: a reset to the new GPS position should be done
	const Vector3f estimated_position = _ekf->getPosition();
	EXPECT_TRUE(isEqual(estimated_position,
			    previous_position + simulated_position_change, 1e-2f));
}

TEST_F(EkfGpsTest, gpsHgtToBaroFallback)
{
	// GIVEN: EKF that fuses GPS and flow, and in GPS height mode
	_sensor_simulator._flow.setData(_sensor_simulator._flow.dataAtRest());
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();

	_ekf_wrapper.enableGpsHeightFusion();

	_sensor_simulator.runSeconds(1);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// WHEN: stopping GPS fusion
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// THEN: the height source should automatically change to baro
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}

TEST_F(EkfGpsTest, altitudeDrift)
{
	// GIVEN: a drifting GNSS altitude
	const float dt = 0.2f;
	const float height_rate = 0.15f;
	const float duration = 80.f;

	// WHEN: running on ground
	for (int i = 0; i < (duration / dt); i++) {
		_sensor_simulator._gps.stepHeightByMeters(height_rate * dt);
		_sensor_simulator.runSeconds(dt);
	}

	float baro_innov = _ekf->aid_src_baro_hgt().innovation;
	BiasEstimator::status status = _ekf->getBaroBiasEstimatorStatus();

	printf("baro innov = %f\n", (double)baro_innov);
	printf("bias: %f, innov bias = %f\n", (double)status.bias, (double)status.innov);

	// THEN: the baro and local position should follow it
	EXPECT_LT(fabsf(baro_innov), 0.1f);
}
