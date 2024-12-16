#ifndef PARACHUTE_PUBLISHER_HPP
#define PARACHUTE_PUBLISHER_HPP

#include "UavcanPublisherBase.hpp"
#include <uavcan/protocol/debug/KeyValue.hpp>
//#include <uavcannode/Subscribers/ArmingStatus.hpp>
#include <uavcan/safety/ArmingStatus.hpp>
//#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/failsafe_injection_command.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/SubscriptionCallback.hpp>
#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
//#include <commander_helper.h>
#include <lib/tunes/tunes.h>

/*
//TODO: Add sanity check for altitude
//TODO: Use 3-secs average height for parachute deployment
//TODO: Make  px4 recognize the device as parachute, not generic can-flow
//TODO: Optimize messages: remove heartbeat, make prearm_failure do it's job as it's published regularly
TODO: Make crash dump after parachute is released
TODO: Trigger
TODO: Buzzer generation
*/

namespace uavcannode
{

class ParachutePublisher :
        public UavcanPublisherBase,
        public uORB::SubscriptionCallbackWorkItem,
        public ModuleParams
{
public:
    ParachutePublisher(px4::WorkItem *work_item, uavcan::INode& node, ModuleParams *parent = nullptr)
        : UavcanPublisherBase(uavcan::protocol::debug::KeyValue::DefaultDataTypeID),
          uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(actuator_armed)),
          ModuleParams(parent),
          _key_value_sub(node),
          _arming_status_sub(node),
          _vehicle_status_sub(ORB_ID(vehicle_status)),
          //_heartbeat_pub(node),
          _failure_pub(node),
          _log_pub(node),
          _restart_timer(node),
          //_heartbeat_timer(node),
          _prearm_failure_timer(node)
          //_buzzer_timer(node)
    {
        _key_value_sub.start(KeyValueCallbackBinder(this, &ParachutePublisher::handleKeyValueHeartbeat));
        _arming_status_sub.start(ArmingStatusCallbackBinder(this, &ParachutePublisher::handleArmingStatusMessage));
        //_heartbeat_timer.setCallback(TimerCbBinder(this, &ParachutePublisher::periodicPublishHeartbeat));
        //_heartbeat_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100)); // Publish heartbeat every second
        _prearm_failure_timer.setCallback(TimerCbBinder(this, &ParachutePublisher::periodicPublishPrearmFailureStatus));
        //_buzzer_timer.setCallback(TimerCbBinder(this, &ParachutePublisher::doBeep));
        _prearm_failure_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100)); // Publish prearm failure status every second
    }

    int init()
    {
        //PX4_DEBUG("ParachutePublisher initialized successfully");
        //_heartbeat_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        //_heartbeat_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        PX4_DEBUG("ParachutePublisher fail pub initialized successfully");
        _failure_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        _failure_pub.setPriority(uavcan::TransferPriority::OneLowerThanHighest);

        PX4_DEBUG("ParachutePublisher log pub initialized successfully");
        _log_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        _log_pub.setPriority(uavcan::TransferPriority::Default);

        uavcan::protocol::debug::LogMessage log_msg;

        if(!GPIO_PARACHUTE || !GPIO_SAFETY || !GPIO_PYRO)
        {
            log_msg.source = "ParachutePublisher";
            log_msg.text = "One of the pins is not defined";
            log_msg.level.value = uavcan::protocol::debug::LogLevel::INFO;
            _log_pub.broadcast(log_msg);
            return 1;
        }

        px4_arch_configgpio(GPIO_SAFETY);
        px4_arch_configgpio(GPIO_PYRO);
        BroadcastAnyUpdates();
        return 0;
    }

void BroadcastAnyUpdates()
{
    int read = px4_arch_gpioread(GPIO_SAFETY);
    int pyro_read = px4_arch_gpioread(GPIO_PYRO);

    // Update counters based on the current read value
    if (read == 1) {
        safety_counter_on++;
        safety_counter_off = 0;  // Reset the off counter
    } else {
        safety_counter_off++;
        safety_counter_on = 0;  // Reset the on counter
    }

    if (pyro_read == 1) {
        pyro_counter_on++;
        pyro_counter_off = 0;  // Reset the off counter
    } else {
        pyro_counter_off++;
        pyro_counter_on = 0;  // Reset the on counter
    }

    // Update safety status and pyro status if we have 3 consistent readings
    if (safety_counter_on >= 3) {
        safety_status = true;
    } else if (safety_counter_off >= 3) {
        safety_status = false;
    }

    if (pyro_counter_on >= 3) {
        pyro_status = true;
    } else if (pyro_counter_off >= 3) {
        pyro_status = false;
    }

    // Calculate average altitude over the last 3 seconds

    if(!armed_status && safety_status && !pyro_status && !was_armed)   //  No pyro && safety button not removed
    {   if (!prearm_failure_triggered) {
            triggerPrearmFailure(true, "Safety button && Pyro");
            prearm_failure_triggered = true;
        }
        safetyBeep(2000, 0, parachute_freq.get());
    }
    else if (!armed_status && !pyro_status && !safety_status && !was_armed) // No pyro
    {
        if (!prearm_failure_triggered) {
            triggerPrearmFailure(true, "Pyro not inserted");
            prearm_failure_triggered = true;
        }
        safetyBeep(1000, 500, parachute_freq.get());
    }
    else if (!armed_status && pyro_status && safety_status && !was_armed)   //Safety button not removed
    {
        if (!prearm_failure_triggered) {
            triggerPrearmFailure(true, "Safety button not removed");
            prearm_failure_triggered = true;
        }
        safetyBeep(500, 500, parachute_freq.get());
    }
    else if (!armed_status && pyro_status && !safety_status && !was_armed && prearm_failure_triggered)  //Both safety button removed and pyro inserted
    {
        triggerPrearmFailure(false, "Safety button && Pyro cleared");
        prearm_failure_triggered = false;
        //px4_arch_unconfiggpio(GPIO_BUZZER);
        //_buzzer_timer.stop();
    }
    else if(!pyro_status && parachute_triggered)   // Pyro exploded as the parachute was triggered
    {
        uavcan::protocol::debug::LogMessage log_msg;
        log_msg.source = "ParachutePublisher";
        log_msg.text = "Pyro exploded";
        log_msg.level.value = uavcan::protocol::debug::LogLevel::WARNING;
    }
    else if(armed_status && !pyro_status && !parachute_triggered)   // Something happened with pyro during flight
    {
        uavcan::protocol::debug::LogMessage log_msg;
        log_msg.source = "ParachutePublisher";
        log_msg.text = "Pyro is not available";
        log_msg.level.value = uavcan::protocol::debug::LogLevel::ERROR;
    }
    else if(!armed_status && !safety_status && was_armed)    // Safety button not inserted after flight
    {
        safetyBeep(500, 500);
    }
    else if(!armed_status && safety_status && was_armed)     // Safety button was inserted back
    {
        //px4_arch_unconfiggpio(GPIO_BUZZER);
        //_buzzer_timer.stop();
    }

    float avg_altitude = calculateAverageAltitude();

    //  Connection errors
    if (!armed_status && !_restart_in_progress && _should_inject_parachute()) {
        if (!connection_lost) {
            triggerPrearmFailure(true, "Connection with FC lost");
            connection_lost = true;
        }
        _restart_in_progress = true;
        _restart_attempts_remaining = 3;
        _restart_timer.setCallback(TimerCbBinder(this, &ParachutePublisher::restartConnectionAttempt));
        _restart_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000)); // Retry connection every second
    } else if (armed_status && _should_inject_parachute()) {
        if(avg_altitude > parachute_min_alt.get())
        {
            triggerFlightTermination("Connection with FC lost");
            flight_term = true;
            //stopTimers();
        }
        else
        {
            uavcan::protocol::debug::LogMessage log_msg;
            log_msg.source = "ParachutePublisher";
            log_msg.text = "Altitude too low for triggering parachute";
            log_msg.level.value = uavcan::protocol::debug::LogLevel::WARNING;
            _log_pub.broadcast(log_msg);
        }
    }
    if(flight_term)
    {
        px4_arch_configgpio(GPIO_PARACHUTE);
    }
}

    void PrintInfo() override
    {
        PX4_INFO("ParachutePublisher: Heartbeat and failsafe communication active.");
    }

private:
    typedef uavcan::MethodBinder<ParachutePublisher *, void (ParachutePublisher::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>&)> KeyValueCallbackBinder;
    typedef uavcan::MethodBinder<ParachutePublisher *, void (ParachutePublisher::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus>&)> ArmingStatusCallbackBinder;
    typedef uavcan::MethodBinder<ParachutePublisher *, void (ParachutePublisher::*)(const uavcan::TimerEvent &)> TimerCbBinder;
    uavcan::Subscriber<uavcan::protocol::debug::KeyValue, KeyValueCallbackBinder> _key_value_sub;
    uavcan::Subscriber<uavcan::equipment::safety::ArmingStatus, ArmingStatusCallbackBinder> _arming_status_sub;
    uORB::Subscription _vehicle_status_sub;
    //uavcan::Publisher<uavcan::protocol::debug::KeyValue> _heartbeat_pub;
    uavcan::Publisher<uavcan::protocol::debug::KeyValue> _failure_pub;
    uavcan::Publisher<uavcan::protocol::debug::LogMessage> _log_pub;
    uavcan::TimerEventForwarder<TimerCbBinder> _restart_timer;
    //uavcan::TimerEventForwarder<TimerCbBinder> _heartbeat_timer;
    uavcan::TimerEventForwarder<TimerCbBinder> _prearm_failure_timer;
    //uavcan::TimerEventForwarder<TimerCbBinder> _buzzer_timer;
    uavcan::MonotonicTime beep_time = _arming_status_sub.getNode().getMonotonicTime() - uavcan::MonotonicDuration::fromMSec(1000);
    uavcan::MonotonicTime _last_heartbeat_time;
    bool _fc_heartbeat_received = true;
    orb_advert_t _failsafe_pub = nullptr;
    orb_advert_t _tune_control_pub = nullptr;
    bool prearm_failure_triggered = false;
    bool _restart_in_progress = false;
    bool armed_status = false;
    bool safety_status = true;
    bool pyro_status = true;
    bool beep = true;
    bool was_armed = false;
    bool parachute_triggered = false;
    bool flight_term = false;
    bool connection_lost = false;
    int _restart_attempts_remaining = 0;
    int previous_value = 1;
    int safety_counter_on = 0;
    int safety_counter_off = 0;
    int pyro_counter_on = 0;
    int pyro_counter_off = 0;
    float altitude = 0.0f;
    float altitude_sum = 0.0f;
    int altitude_count = 0;
    int _beep_frequency = 2000;       // Default frequency in Hz
    int _beep_duration_ms = 0;       // Remaining beep duration in milliseconds
    bool _buzzer_state = false;      // Current state of the buzzer (on/off)
    Tunes _tunes {};

    const int ALTITUDE_WINDOW_SIZE = 30; // For 3-second average at 10 Hz

    DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CANNODE_PCHT_MIN>) parachute_min_alt,
        (ParamFloat<px4::params::CANNODE_PCHT_MAX>) parachute_max_alt,
        (ParamInt<px4::params::CANNODE_PCHT_FQ>) parachute_freq
	    );

    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    void parameters_update()
    {
        if (_parameter_update_sub.updated()) {
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);

            // If any parameter updated, call updateParams() to check if
            // this class attributes need updating (and do so).
            updateParams();
        }
    }

    bool altitudeSanityCheck(float alt)
    {
        return (alt < parachute_max_alt.get());
    }

    void addAltitude(float alt)
    {
        if (altitude_count >= ALTITUDE_WINDOW_SIZE) {
            // Subtract the oldest altitude value (average adjustment)
            altitude_sum -= altitude_sum / ALTITUDE_WINDOW_SIZE;
        } else {
            altitude_count++;
        }
        altitude_sum += alt;
    }

    float calculateAverageAltitude()
    {
        if (altitude_count == 0) {
            return 0.0f;
        }
        return altitude_sum / static_cast<float>(altitude_count);
    }

    void publishBeep(uint16_t frequency, uint32_t duration, uint32_t silence, uint8_t volume = 10) {
        // Prepare the tune_control message
        //_tunes.set_string("MNT140L8O4EGEGAC<GE>C<DGEFGE>C<EGEF>GABAGF<GE>F<GABAC>DE<G>C<EGEGAC<GE>C<DGEF", tune_control_s::VOLUME_LEVEL_DEFAULT);
        //tune_control_s tune_control{};
        tune_control_s tune_control{};
        tune_control.frequency = frequency;       // Frequency in Hz
        tune_control.duration = duration * 1000;         // Duration in milliseconds
        tune_control.silence = silence;           // Silence between tones in milliseconds
        tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;             // Volume (range: 0-10)
        tune_control.tune_id = 0;
        tune_control.tune_override = true;
        tune_control.timestamp = hrt_absolute_time();

        /*Tunes::ControlResult ret = _tunes.set_control(tune_control);
        uavcan::protocol::debug::LogMessage log_msg;

		if (ret == Tunes::ControlResult::InvalidTune) {
			PX4_WARN("Tune ID not recognized.");
            log_msg.source = "ParachutePublisher";
            log_msg.text = "Tune ID not recognized.";
            log_msg.level.value = uavcan::protocol::debug::LogLevel::INFO;
            _log_pub.broadcast(log_msg);
		}
        if (ret == Tunes::ControlResult::Success) {
			PX4_WARN("Success");
		}*/
        //PX4_INFO("Something");

        // Publish the message to the uORB topic
        //if (_tune_control_pub == nullptr) {
            _tune_control_pub = orb_advertise(ORB_ID(tune_control), &tune_control);
        //} else {
            orb_publish(ORB_ID(tune_control), _tune_control_pub, &tune_control);
        //}

        uORB::SubscriptionData<tune_control_s> tune_control_sub{ORB_ID(tune_control)};

        //float avg_altitude = calculateAverageAltitude();
        //char buffer[60];
        //log_msg.source = "ParachutePublisher";
        PX4_INFO("freq: %d, dur: %d, sil: %d, vol: %d", tune_control_sub.get().frequency, static_cast<int>(tune_control_sub.get().duration), static_cast<int>(tune_control_sub.get().silence), tune_control_sub.get().volume);
        //log_msg.text = buffer;
        //log_msg.level.value = uavcan::protocol::debug::LogLevel::INFO;
        //_log_pub.broadcast(log_msg);

    }


    void safetyBeep(uint16_t beep_duration, uint16_t idle_duration, uint16_t frequency = 1000, uint8_t volume = 10) {
        static bool _beep = true;

        if (_beep) {
            // Start a beep with the specified parameters
            publishBeep(frequency, beep_duration, idle_duration, volume);
            //tune_neutral(true);
            //_tunes.set_string("MNT140L8O4EGEGAC<GE>C<DGEFGE>C<EGEF>GABAGF<GE>F<GABAC>DE<G>C<EGEGAC<GE>C<DGEF", tune_control_s::VOLUME_LEVEL_DEFAULT);
            _beep = false;
        } else {
            // Wait for the idle duration before toggling beep state again
            uavcan::MonotonicTime current_time = _arming_status_sub.getNode().getMonotonicTime();
            if (current_time - beep_time > uavcan::MonotonicDuration::fromMSec(beep_duration + idle_duration)) {
                _beep = true; // Toggle the beep state
                beep_time = current_time;
            }
        }
    }




    void handleKeyValueHeartbeat(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>& msg)
    {
        if (msg.key == "FC_heartbeat") {
            _last_heartbeat_time = _key_value_sub.getNode().getMonotonicTime();
            PX4_DEBUG("Received FC heartbeat through KeyValue");
            _fc_heartbeat_received = true;

            if(altitudeSanityCheck(msg.value))
            {
                altitude = msg.value;
                addAltitude(altitude);
                PX4_DEBUG("Received altitude through KeyValue");
            }
            else PX4_WARN("Received insane altitude value through KeyValue");

            if (connection_lost) {
                connection_lost = false;
                triggerPrearmFailure(false, "Connection regained.");
            }

            if (_restart_in_progress) {
                _restart_timer.stop();
                _restart_in_progress = false;
                PX4_INFO("Connection reestablished. Restart process stopped.");
            }
        }
        if(msg.key == "Flightterm")
        {
            triggerFlightTermination("External parachute command received");
            flight_term = true;
            //stopTimers();
        }
    }

    void handleArmingStatusMessage(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus>& msg)
    {
        if(msg.status == 0)
        {
            armed_status = false;
            px4_arch_unconfiggpio(GPIO_nLED_BLUE);
        }
        else
        {
            armed_status = true;
            was_armed = true;
            px4_arch_configgpio(GPIO_nLED_BLUE);
        }
    }

    /*void periodicPublishHeartbeat(const uavcan::TimerEvent &)
    {
        sendHeartbeat();
    }*/


    void periodicPublishPrearmFailureStatus(const uavcan::TimerEvent &)
    {
        uavcan::protocol::debug::KeyValue prearm_failure_msg;
        prearm_failure_msg.key = flight_term ? "flight_term" : "prearm_failure";
        prearm_failure_msg.value = prearm_failure_triggered ? 1.0f : 0.0f;

        /*uavcan::protocol::debug::LogMessage log_msg;
        float avg_altitude = calculateAverageAltitude();
        char buffer[60];
        log_msg.source = "ParachutePublisher";
        sprintf(buffer, "Pyro status: %d, Altitude: %f", pyro_status, static_cast<double>(avg_altitude));
        log_msg.text = buffer;
        log_msg.level.value = uavcan::protocol::debug::LogLevel::INFO;
        _log_pub.broadcast(log_msg);*/

        int res = _failure_pub.broadcast(prearm_failure_msg);
        if (res < 0) {
            PX4_ERR("Failed to broadcast Prearm failure status");
        }
    }

    bool _should_inject_parachute()
    {
        uavcan::MonotonicTime current_time = _key_value_sub.getNode().getMonotonicTime();
        if ((current_time - _last_heartbeat_time) > uavcan::MonotonicDuration::fromMSec(3000)) {
            _fc_heartbeat_received = false;
        }
        return !_fc_heartbeat_received;
    }

    /*void sendHeartbeat()
    {
        uavcan::protocol::debug::KeyValue heartbeat_msg;
        heartbeat_msg.key = flight_term ? "flight_term" : "parachute_ready";
        heartbeat_msg.value = 1.0f;

        int res = _heartbeat_pub.broadcast(heartbeat_msg);
        if (res < 0) {
            PX4_ERR("Failed to broadcast Parachute heartbeat");
        } else {
            PX4_DEBUG("Parachute heartbeat sent successfully");
        }
    }*/

    void triggerPrearmFailure(bool status, const char* cause)
    {
        const char* status_string = status ? "Triggering" : "Clearing";
        PX4_WARN("%s prearm failure: %s", status_string, cause);

        uavcan::protocol::debug::KeyValue heartbeat_msg;

        heartbeat_msg.key = "prearm_failure";
        heartbeat_msg.value = status ? 1.0f : 0.0f;

        for(int i = 0; i < 5; i++)
        {
            _failure_pub.broadcast(heartbeat_msg);
        }
    }

    void triggerFlightTermination(const char* cause)
    {
        PX4_ERR("Triggering flight termination: %s", cause);

        uavcan::protocol::debug::KeyValue heartbeat_msg;

        heartbeat_msg.key = "flight_term";
        heartbeat_msg.value = 1.0f;

        for(int i = 0; i < 10; i++)
        {
            _failure_pub.broadcast(heartbeat_msg);
        }
        //usleep(100000);
    }

    void restartConnectionAttempt(const uavcan::TimerEvent &)
    {
        if (_restart_attempts_remaining > 0) {
            PX4_WARN("Attempting to reestablish connection... (%d attempts remaining)", _restart_attempts_remaining);
            _restart_attempts_remaining--;
        } else {
            PX4_ERR("Failed to reestablish connection after multiple attempts. Aborting.");
            _restart_timer.stop();
            _restart_in_progress = false;
        }
    }

    void stopTimers()
    {
        PX4_DEBUG("Stopping timers after flight termination.");
        _key_value_sub.stop();
        _restart_timer.stop();
        //_heartbeat_timer.stop();
        _prearm_failure_timer.stop();
    }
};

} // namespace uavcannode

#endif // PARACHUTE_PUBLISHER_HPP
