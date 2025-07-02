#ifndef EKF_FLOW_H
#define EKF_FLOW_H

#include "sensor.h"

namespace sensor_simulator
{
namespace sensor
{

class Flow: public Sensor
{
public:
	Flow(std::shared_ptr<Ekf> ekf);
	~Flow();

	void setData(const flowSample &flow);
	flowSample dataAtRest();

private:
	flowSample _flow_data;

	void send(uint64_t time) override;

};

} // namespace sensor
} // namespace sensor_simulator
#endif // !EKF_FLOW_H
