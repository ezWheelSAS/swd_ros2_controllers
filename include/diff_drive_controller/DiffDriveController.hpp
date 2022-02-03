/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "diff_drive_controller/DiffDriveParameters.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ezw {
    namespace swd {

        /**
         * @brief Differential Drive Controller for ez-Wheel Gen2 wheels
         * It subscribes to the `/node/set_speed` or `/node/cmd_vel` topics:
         * - `/node/set_speed` of type `geometry_msgs::Point`: The `x`, `y`
         *   components of the `/set_speed` message
         *   represents respectively the left and right motor speed in (rad/s)
         * - `/node/cmd_vel` of type `geometry_msgs::Twist`: The linear and angular
         *   velocities.
         * The controller publishes the odometry to `/node/odom` and TFs, the safety
         * functions to `/node/safety`.
         * 
         * DiffDriveController inheriting from rclcpp_lifecycle::LifecycleNode
         * 
         * The lifecycle talker does not like the regular "talker" node
         * inherit from node, but rather from lifecyclenode. This brings
         * in a set of callbacks which are getting invoked depending on
         * the current state of the node.
         * Every lifecycle node has a set of services attached to it
         * which make it controllable from the outside and invoke state
         * changes.
         */
        class DiffDriveController : public rclcpp_lifecycle::LifecycleNode {
           public:
            /**
             * @brief Construct a new Diff Drive Controller object
             * 
             * @param p_node_name Node name
             * @param p_intra_process_comms  
             */
            explicit DiffDriveController(const std::string &p_node_name, std::shared_ptr<DiffDriveParameters> p_params, bool p_intra_process_comms = false);

            /**
             * @brief 
             * 
             */
            void publish();

            /// Transition callback for state configuring
            /**
             * on_configure callback is being called when the lifecycle node
             * enters the "configuring" state.
             * Depending on the return value of this function, the state machine
             * either invokes a transition to the "inactive" state or stays
             * in "unconfigured".
             * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
             * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
             * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
             */
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

            /// Transition callback for state activating
            /**
           * on_activate callback is being called when the lifecycle node
           * enters the "activating" state.
           * Depending on the return value of this function, the state machine
           * either invokes a transition to the "active" state or stays
           * in "inactive".
           * TRANSITION_CALLBACK_SUCCESS transitions to "active"
           * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
           * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
           */
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
            {
                // The parent class method automatically transition on managed entities
                // (currently, LifecyclePublisher).
                // m_pub->on_activate() could also be called manually here.
                // Overriding this method is optional, a lot of times the default is enough.
                LifecycleNode::on_activate(state);

                RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

                // Let's sleep for 2 seconds.
                // We emulate we are doing important
                // work in the activating phase.
                std::this_thread::sleep_for(2s);

                // We return a success and hence invoke the transition to the next
                // step: "active".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "inactive" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

            /// Transition callback for state deactivating
            /**
           * on_deactivate callback is being called when the lifecycle node
           * enters the "deactivating" state.
           * Depending on the return value of this function, the state machine
           * either invokes a transition to the "inactive" state or stays
           * in "active".
           * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
           * TRANSITION_CALLBACK_FAILURE transitions to "active"
           * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
           */
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
            {
                // The parent class method automatically transition on managed entities
                // (currently, LifecyclePublisher).
                // m_pub->on_deactivate() could also be called manually here.
                // Overriding this method is optional, a lot of times the default is enough.
                LifecycleNode::on_deactivate(state);

                RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

                // We return a success and hence invoke the transition to the next
                // step: "inactive".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "active" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

            /// Transition callback for state cleaningup
            /**
           * on_cleanup callback is being called when the lifecycle node
           * enters the "cleaningup" state.
           * Depending on the return value of this function, the state machine
           * either invokes a transition to the "unconfigured" state or stays
           * in "inactive".
           * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
           * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
           * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
           */
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
            {
                // In our cleanup phase, we release the shared pointers to the
                // timer and publisher. These entities are no longer available
                // and our node is "clean".
                m_timer.reset();
                m_pub.reset();

                RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

                // We return a success and hence invoke the transition to the next
                // step: "unconfigured".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "inactive" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

            /// Transition callback for state shutting down
            /**
             * on_shutdown callback is being called when the lifecycle node
             * enters the "shuttingdown" state.
             * Depending on the return value of this function, the state machine
             * either invokes a transition to the "finalized" state or stays
             * in its current state.
             * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
             * TRANSITION_CALLBACK_FAILURE transitions to current state
             * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
             */
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
            {
                // In our shutdown phase, we release the shared pointers to the
                // timer and publisher. These entities are no longer available
                // and our node is "clean".
                m_timer.reset();
                m_pub.reset();

                RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

                // We return a success and hence invoke the transition to the next
                // step: "finalized".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the current state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

           private:
            // We hold an instance of a lifecycle publisher. This lifecycle publisher
            // can be activated or deactivated regarding on which state the lifecycle node
            // is in.
            // By default, a lifecycle publisher is inactive by creation and has to be
            // activated to publish messages into the ROS world.
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> m_pub;

            // We hold an instance of a timer which periodically triggers the publish function.
            // As for the beta version, this is a regular timer. In a future version, a
            // lifecycle timer will be created which obeys the same lifecycle management as the
            // lifecycle publisher.
            std::shared_ptr<rclcpp::TimerBase> m_timer;

            std::shared_ptr<DiffDriveParameters> m_params;
        };

    }  // namespace swd
}  // namespace ezw
