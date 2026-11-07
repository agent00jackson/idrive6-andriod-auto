// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#include "roadbook_models.hpp"
#include "kombi_nav_test_util.hpp"
#include <assert.h>
#include <stdio.h>

int main() {
    using namespace nbt;
    using namespace kntest;

    std::vector<TLane> lanes;
    lanes.push_back(lane(1, false));
    lanes.push_back(lane2(1, false, 5, true));
    const Bytes state = navStateLanes(8, "MAIN ST", lanes);
    const Bytes position = curPos(150, "500", 6);
    kombinav::NavMessages messages;
    messages.store(0x8006, &state[0], state.size());
    messages.store(0x8007, &position[0], position.size());
    kombinav::NavView view;
    assert(kombinav::viewFromMessages(messages, view));

    const std::string create = roadbook::guidanceModels(view, true);
    const std::string update = roadbook::guidanceModels(view, false);
    assert(create.find("\"appId\":\"BMW.HMI.Roadbook\"") != std::string::npos);
    assert(update.find("\"msg\":\"create\"") == std::string::npos);
    assert(update.find("\"laneType\":\"BEST_LANE\",\"arrows\":[\"STRAIGHT_ON\",\"RIGHT\"]") !=
           std::string::npos);
    assert(update.find("\"direction\":\"RIGHT\"") != std::string::npos);
    assert(update.find("\"laneGuidings\":[452984832]") != std::string::npos);
    assert(update.find("setDataLg") < update.find("setDataMa"));
    assert(update.find("setDataML") < update.find("setDataMaCtrl"));
    assert(roadbook::clearModels().find("\"activeList\":[]") != std::string::npos);

    view.lanes[0][0].shape = 0;
    assert(roadbook::guidanceModels(view, false).find("setDataLg") == std::string::npos);
    view.aa_type = 0;
    assert(roadbook::guidanceModels(view, false).empty());
    printf("test_roadbook_models OK\n");
}
