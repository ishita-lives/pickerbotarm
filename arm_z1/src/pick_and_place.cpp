#include "unitree_arm_sdk/control/unitreeArm.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace UNITREE_ARM;

class PickAndPlace {
public:
    PickAndPlace() {
        arm_ = std::make_shared<unitreeArm>(true);
        // enable gripper

        // ensure motions are blocking until finished
        arm_->setWait(true);

        // transition to Passive
        if (!arm_->setFsm(ArmFSMState::PASSIVE)) {
            std::cerr << "Failed to set FSM to Passive" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // transition to Move
        if (!arm_->setFsm(ArmFSMState::MOVEJ)) {
            std::cerr << "Failed to set FSM to Operate" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    bool executePickAndPlace(const Vec6& pick_joints, const Vec6& place_joints) {
        std::cout << "Moving to pick position..." << std::endl;
        if (!arm_->MoveJ(pick_joints, 0.5)) {
            std::cerr << "Failed to move to pick position!" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        // wait to stabilise

        std::cout << "Closing gripper..." << std::endl;
        arm_->setGripperCmd(-M_PI / 2, 0.6);
        // close with effort
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Moving to place position..." << std::endl;
        if (!arm_->MoveJ(place_joints, 0.5)) {
            std::cerr << "Failed to move to place position!" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        // wait to stabilise

        std::cout << "Opening gripper..." << std::endl;
        arm_->setGripperCmd(0.0, 0.6);
        // open with effort
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Pick-and-place completed!" << std::endl;
        return true;
    }

private:
    std::shared_ptr<unitreeArm> arm_;
};

int main() {
    // joint angles in radians
    Vec6 pick_joints = {0.0, -0.5, 0.3, -1.0, 0.0, 0.0};
    Vec6 place_joints = {0.5, -0.3, 0.2, -1.2, 0.1, 0.0};

    PickAndPlace controller;
    if (!controller.executePickAndPlace(pick_joints, place_joints)) {
        std::cerr << "Operation failed!" << std::endl;
        return -1;
    }

    return 0;
}