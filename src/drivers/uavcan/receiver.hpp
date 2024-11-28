#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <px4_platform_common/log.h>
#include <uORB/topics/failsafe_injection_command.h>
#include <uORB/topics/button_event.h>
#include <uORB/topics/vehicle_local_position.h>
#include <cstdlib>  // For std::rand()
#include <ctime>    // For time tracking
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>

/*
TODO: Add custom notifications
TODO: Add external parachute command and make commander use it
!   Altitude
*/

// Initialize the parachute status timeout duration (e.g., 5 seconds)
const uavcan::MonotonicDuration PARACHUTE_TIMEOUT = uavcan::MonotonicDuration::fromMSec(3000);

class ParachuteReceiver : public ModuleParams
{
    public:
        ParachuteReceiver(uavcan::INode &node, ModuleParams *parent = nullptr) :
            ModuleParams(parent),
            _node(node),
            _kv_pub(node),    // KeyValue publisher
            _kv_sub(node),    // KeyValue subscriber
            _timer(node),     // Timer for periodic publishing
            _watchdog_timer(node) // Timer for monitoring parachute status
        {
        }

        void check_parachute_timeout(const uavcan::TimerEvent &)
        {
            uavcan::MonotonicTime current_time = _node.getMonotonicTime();
            if ((current_time - _last_parachute_time) > PARACHUTE_TIMEOUT) {
                PX4_ERR("Parachute status timeout! Parachute may not be ready.");
                fail_msg.timestamp = hrt_absolute_time();
                fail_msg.inject_failsafe = false;
                fail_msg.inject_prearm_failure = true;
                fail_msg.failure_type = 1;
                fail_msg.severity = 1;

                _failsafe_injection_command_pub = orb_advertise(ORB_ID(failsafe_injection_command), &fail_msg);
                bool pub_res = (orb_publish(ORB_ID(failsafe_injection_command), _failsafe_injection_command_pub, &fail_msg) == PX4_OK);
                if (pub_res) {
                    PX4_WARN("Sucessfully published prearm failure via uORB");
                } else {
                    PX4_ERR("Failed to publish failsafe injection command via uORB.");
                }

                prearm_failure_status = true;
            }


        }

        // Initialize the publishers, subscribers, and timers
        int init()
        {
            initializeSafetyButton();

            int32_t uavcan_sub_parachute = 0;
            param_get(param_find("UAVCAN_SUB_PRCHT"), &uavcan_sub_parachute);
            if(uavcan_sub_parachute == 0)
            {
                return 0;
            }

            // Initialize KeyValue publisher
            _kv_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(100));
            _kv_pub.setPriority(uavcan::TransferPriority::MiddleLower);

            // Set up the subscriber callback
            _kv_sub.start(KeyValueCallbackBinder(this, &ParachuteReceiver::handleKeyValueMessage));

            // Start the periodic publishing timer (every second)
            _timer.setCallback(TimerCbBinder(this, &ParachuteReceiver::periodic_publish));
            _timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));

            // Start the watchdog timer to check for parachute status timeout
            _watchdog_timer.setCallback(TimerCbBinder(this, &ParachuteReceiver::check_parachute_timeout));
            _watchdog_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));

            // Initialize the last parachute status time
            _last_parachute_time = _node.getMonotonicTime();


            PX4_INFO("ParachuteReceiver initialized successfully");
            return 0;
        }

    private:
        uavcan::INode &_node;
        uavcan::Publisher<uavcan::protocol::debug::KeyValue> _kv_pub;

        // Subscriber for KeyValue messages
        typedef uavcan::MethodBinder<ParachuteReceiver *,
            void (ParachuteReceiver::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>&)>
            KeyValueCallbackBinder;
        uavcan::Subscriber<uavcan::protocol::debug::KeyValue, KeyValueCallbackBinder> _kv_sub;

        // Timer for periodic publishing
        typedef uavcan::MethodBinder<ParachuteReceiver *, void (ParachuteReceiver::*)(const uavcan::TimerEvent &)>
            TimerCbBinder;
        uavcan::TimerEventForwarder<TimerCbBinder> _timer;

        // Timer for monitoring parachute status timeout
        uavcan::TimerEventForwarder<TimerCbBinder> _watchdog_timer;
        float altitude = 0.0f;

        // Time of the last received parachute status
        uavcan::MonotonicTime _last_parachute_time;
        orb_advert_t _failsafe_injection_command_pub;
        failsafe_injection_command_s fail_msg;
        int flight_termination_counter = 0;
        bool parachute_ready = false;
        bool prev_prearm_failure_status = false;
        bool prearm_failure_status = false;
        bool  flightterm = false;
        //int prearm_failure_counter = 0;

        DEFINE_PARAMETERS(
		(ParamFloat<px4::params::UAVCAN_PRCHT_ALT>) _parachute_min_alt
	    )

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

        // Periodic publishing of 'FC_heartbeat' message
        void periodic_publish(const uavcan::TimerEvent &)
        {
            uORB::SubscriptionData<vehicle_local_position_s> vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
            altitude = -(vehicle_local_position_sub.get().z);


            uORB::SubscriptionData<vehicle_command_s> vehicle_command_sub{ORB_ID(vehicle_command)};
            if(vehicle_command_sub.get().command == vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE && static_cast<int>(vehicle_command_sub.get().param1) == static_cast<int>(vehicle_command_s::PARACHUTE_ACTION_RELEASE))
            {
                PX4_ERR("Triggering parachute: External command received");
                flightterm = true;

                //_timer.stop();
                //_watchdog_timer.stop();

            }

            //vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	        //vcmd.param1 = static_cast<float>(vehicle_command_s::PARACHUTE_ACTION_RELEASE);

            // Publish 'FC_heartbeat' KeyValue message
            uavcan::protocol::debug::KeyValue kv_msg;
            kv_msg.value = altitude;//1.0f;  // Indicating FC is active
            kv_msg.key = flightterm ? "Flightterm" : "FC_heartbeat";

            const int kv_pub_res = _kv_pub.broadcast(kv_msg);
            if (kv_pub_res < 0) {
                PX4_ERR("KeyValue publication failure: %d", kv_pub_res);
            } else {
                PX4_INFO("FC heartbeat message sent");
            }

            //PX4_INFO("Parameter value: %f", static_cast<double>(_parachute_min_alt.get()));



            //PX4_INFO("Altitude: %f", static_cast<double>(altitude));
            /*            uavcan::protocol::debug::KeyValue alt_msg;
            kv_msg.value = altitude;
            kv_msg.key = "Altitude";

            const int alt_pub_res = _kv_pub.broadcast(alt_msg);
            if (alt_pub_res < 0) {
                PX4_ERR("Altitude publication failure: %d", alt_pub_res);
            } else {
                PX4_INFO("Altitude message sent");
            }*/
        }

        // Handle incoming KeyValue messages
        void handleKeyValueMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg)
        {
            uORB::SubscriptionData<failsafe_injection_command_s> failsafe_injection_sub{ORB_ID(failsafe_injection_command)};
            prearm_failure_status = failsafe_injection_sub.get().inject_prearm_failure;
            //if(failsafe_injection_sub.get().inject_failsafe) PX4_WARN("Flight termination received via uORB");


            if (msg.key == "parachute_ready") {
                // Update the last parachute status time
                _last_parachute_time = _node.getMonotonicTime();
                PX4_INFO("Received parachute ready message");

                if(!parachute_ready)
                {
                    fail_msg.timestamp = hrt_absolute_time();
                    fail_msg.inject_failsafe = false;
                    fail_msg.inject_prearm_failure = false;
                    fail_msg.failure_type = 1;
                    fail_msg.severity = 1;
                    parachute_ready = true;

                    _failsafe_injection_command_pub = orb_advertise(ORB_ID(failsafe_injection_command), &fail_msg);

                    bool pub_res = (orb_publish(ORB_ID(failsafe_injection_command), _failsafe_injection_command_pub, &fail_msg) == PX4_OK);
                    if (!pub_res) {
                        PX4_ERR("Failed to publish failsafe injection command via uORB.");
                    }
                }
            }
            if (msg.key == "prearm_failure" )//&& (prev_prearm_failure_status != prearm_failure_status))
            {
                bool prearm_status_changed = (prev_prearm_failure_status != prearm_failure_status);
                if(prearm_status_changed)
                {
                    PX4_WARN("Received prearm failure message");
                    PX4_INFO("Message value: %f", static_cast<double>(msg.value));
                }
                fail_msg.timestamp = hrt_absolute_time();
                if(msg.value > 0.5f)
                {
                    if(prearm_status_changed) PX4_ERR("Prearm failure from parachute");
                    fail_msg.inject_failsafe = false;
                    fail_msg.inject_prearm_failure = true;
                    fail_msg.failure_type = 1;
                    fail_msg.severity = 1;
                }
                else
                {
                    if(prearm_status_changed) PX4_WARN("Prearm failure from parachute cleared");
                    fail_msg.inject_failsafe = false;
                    fail_msg.inject_prearm_failure = false;
                    fail_msg.failure_type = 1;
                    fail_msg.severity = 1;
                }

                //if (_failsafe_injection_command_pub == nullptr) {
                    _failsafe_injection_command_pub = orb_advertise(ORB_ID(failsafe_injection_command), &fail_msg);
                //}

                bool pub_res = (orb_publish(ORB_ID(failsafe_injection_command), _failsafe_injection_command_pub, &fail_msg) == PX4_OK);
                if (pub_res && prearm_status_changed) {
                    PX4_WARN("Sucessfully published prearm failure via uORB");
                } else if (prearm_status_changed) {
                    PX4_ERR("Failed to publish failsafe injection command via uORB.");
                }
            }
            if (msg.key == "flight_termination")
            {
                PX4_WARN("Received prearm failure message");
                flight_termination_counter++;
                if(flight_termination_counter > 6)
                {
                    PX4_ERR("Flight termination from parachute");
                    fail_msg.timestamp = hrt_absolute_time();
                    fail_msg.inject_failsafe = true;
                    fail_msg.inject_prearm_failure = false;
                    fail_msg.failure_type = 1;
	                fail_msg.severity = 1;

                    orb_publish(ORB_ID(failsafe_injection_command), _failsafe_injection_command_pub, &fail_msg);
                    PX4_ERR("Sent prearm failure command via uORB");
                    flight_termination_counter = 0;
                }
            }

            prev_prearm_failure_status = prearm_failure_status;
        }

        void initializeSafetyButton()
        {
            struct button_event_s button_cmd;
            button_cmd.timestamp = hrt_absolute_time();
            button_cmd.triggered = true;

            orb_advert_t button_pub = orb_advertise(ORB_ID(safety_button), &button_cmd);
            bool pub_res = (orb_publish(ORB_ID(safety_button), button_pub, &button_cmd) == PX4_OK);

            if (pub_res) {
                PX4_DEBUG("Init safety_button message sent successfully");
            } else {
                PX4_ERR("Failed to send the init safety_button message");
                //throw std::runtime_error("Safety button initialization failed");
            }
        }
};
