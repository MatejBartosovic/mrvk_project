//
// Created by matejko on 18.3.2019.
//

#include <mrvk_gui/MrvkControllButtons.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
namespace mrvk_gui {
    MrvkControllButtons::MrvkControllButtons() {
        ros::NodeHandle n("/");
        ros::ServiceClient setCentralStopServiceClient = n.serviceClient<std_srvs::Trigger>("reset_central_stop");
        ros::ServiceClient resetCentralStopServiceClient =n.serviceClient<std_srvs::Trigger>("setCentralStop");
        ros::ServiceClient blockMovementServiceClient = n.serviceClient<std_srvs::SetBool>("block_movement");
    }

    void MrvkControllButtons::setCentralStop() {
        std_srvs::Trigger msg;
        if(!setCentralStopServiceClient.call(msg)){
            throw MrvkControllButtonsException("Set central stop service is not running");
        }
        if(!msg.response.success){
            throw MrvkControllButtonsException("Service responded with error: " + msg.response.message);
        }
    }

    void MrvkControllButtons::resetCentralStop() {
        std_srvs::Trigger msg;
        if(!resetCentralStopServiceClient.call(msg)){
            throw MrvkControllButtonsException("Reset central stop service is not running");
        }
        if(!msg.response.success){
            throw MrvkControllButtonsException("Service responded with error: " + msg.response.message);
        }
    }

    void MrvkControllButtons::blockMovement() {
        std_srvs::SetBool msg;
        msg.request.data = true;
        if(!blockMovementServiceClient.call(msg)){
            throw MrvkControllButtonsException("Block movement service is not running");
        }
        if(!msg.response.success){
            throw MrvkControllButtonsException("Service responded with error: " + msg.response.message);
        }
    }

    void MrvkControllButtons::unblockMovement() {
        std_srvs::SetBool msg;
        msg.request.data = false;
        if(!blockMovementServiceClient.call(msg)){
            throw MrvkControllButtonsException("Block movement service is not running");
        }
        if(!msg.response.success){
            throw MrvkControllButtonsException("Service responded with error: " + msg.response.message);
        }
    }
}