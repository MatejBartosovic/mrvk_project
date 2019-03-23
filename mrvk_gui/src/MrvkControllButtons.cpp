//
// Created by matejko on 18.3.2019.
//

#include <mrvk_gui/MrvkControllButtons.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <mrvk_gui/ButtonException.h>
namespace mrvk_gui {
    MrvkControllButtons::MrvkControllButtons() {
        ros::NodeHandle n("/");
        setCentralStopServiceClient = n.serviceClient<std_srvs::Trigger>("reset_central_stop");
        resetCentralStopServiceClient =n.serviceClient<std_srvs::Trigger>("setCentralStop");
        blockMovementServiceClient = n.serviceClient<std_srvs::SetBool>("block_movement");
    }

    void MrvkControllButtons::setCentralStop() {
        std_srvs::Trigger msg;
        callService(setCentralStopServiceClient,msg);
    }

    void MrvkControllButtons::resetCentralStop() {
        std_srvs::Trigger msg;
        callService(resetCentralStopServiceClient,msg);
    }

    void MrvkControllButtons::blockMovement() {
        std_srvs::SetBool msg;
        msg.request.data = true;
        callService(blockMovementServiceClient,msg);
    }

    void MrvkControllButtons::unblockMovement() {
        std_srvs::SetBool msg;
        msg.request.data = false;
        callService(blockMovementServiceClient,msg);

    }
}