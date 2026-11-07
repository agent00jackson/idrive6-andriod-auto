// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#ifndef ROADBOOK_MODELS_HPP
#define ROADBOOK_MODELS_HPP

#include "kombi_nav_msg.hpp"
#include <sstream>
#include <locale>

namespace nbt {
namespace roadbook {

inline const char* directionName(int direction) {
    switch (direction) {
        case 1: return "STRAIGHT_ON";
        case 2: return "HALF_LEFT";
        case 3: return "HALF_RIGHT";
        case 4: return "TIGHT_LEFT";
        case 5: return "TIGHT_RIGHT";
        case 6: return "LEFT";
        case 7: return "RIGHT";
        case 8: return "UTURN_LEFT";
        case 9: return "UTURN_RIGHT";
        default: return NULL;
    }
}

inline const char* maneuverDirection(int type) {
    // Only types for which this first test has a matching native icon.
    switch (type) {
        case 1: case 2: case 36: return "STRAIGHT_ON";
        case 3: case 4: case 5: case 6: case 7: case 8:
        case 9: case 10: case 11: case 12:
            return directionName(kombinav::kombiDirection(type));
        default:
            return NULL;
    }
}

enum ModelId {
    kManeuver = 0x0f000000,
    kLanes = 0x1b000000,
    kDistance = 0x1d000000,
    kList = 0x20000000,
    kListState = 0x21000000,
    kManeuverState = 0x22000000
};

// The diagnostic HTTP backend takes named CommonAPI variants inside the
// message's data array. IDs and type names are from X20511I HmiMain3D.
inline void beginModel(std::ostream& out, const char* type, ModelId id) {
    out << "{\"vType\":\"" << type << "\",\"vData\":{\"modelId\":" << id;
}

inline void listMessage(std::ostream& out, bool active) {
    out << "{\"msg\":\"setDataML\",\"data\":[";
    beginModel(out, "ManeuverList", kList);
    out << ",\"activeFrom\":0,\"activeNum\":" << (active ? 1 : 0)
        << ",\"activeTotal\":" << (active ? 1 : 0) << ",\"activeList\":[";
    if (active) out << kManeuver;
    out << "],\"passiveFrom\":0,\"passiveNum\":0,\"passiveTotal\":0,\"passiveList\":[]}}]}";
}

inline void controlMessage(std::ostream& out, const char* state, bool active) {
    out << "{\"msg\":\"setDataMaCtrl\",\"data\":[";
    beginModel(out, "ManeuverListState", kListState);
    out << ",\"guidanceState\":\"" << state << "\",\"activeManId\":"
        << (active ? static_cast<unsigned>(kManeuver) : 0xffffffffu)
        << ",\"passiveManId\":4294967295,\"eta\":0,\"totalDelay\":0,"
        << "\"routeLength\":0,\"iconID\":4294967295}}]}";
}

inline std::string clearModels() {
    std::ostringstream out;
    out << '[';
    listMessage(out, false);
    out << ',';
    controlMessage(out, "NotActive", false);
    out << ",{\"msg\":\"dispose\",\"data\":null}]";
    return out.str();
}

inline bool validLanes(const std::vector<kombinav::AaLane>& lanes) {
    if (lanes.empty() || lanes.size() > 8) return false;
    for (size_t i = 0; i < lanes.size(); ++i) {
        if (lanes[i].empty() || lanes[i].size() > 9) return false;
        for (size_t j = 0; j < lanes[i].size(); ++j) {
            if (!directionName(kombinav::laneArrow(lanes[i][j].shape))) return false;
        }
    }
    return true;
}

inline void laneMessage(std::ostream& out, const std::vector<kombinav::AaLane>& lanes,
                        double distance) {
    out << "{\"msg\":\"setDataLg\",\"data\":[";
    beginModel(out, "LaneGuiding", kLanes);
    out << ",\"distance\":" << distance << ",\"previewDist\":180,\"range\":120,\"lanes\":[";
    for (size_t i = 0; i < lanes.size(); ++i) {
        if (i) out << ',';
        const kombinav::AaLane& lane = lanes[i];
        bool recommended = false;
        for (size_t j = 0; j < lane.size(); ++j) recommended = recommended || lane[j].highlighted;
        out << "{\"laneType\":\"" << (recommended ? "BEST_LANE" : "POSSIBLE_LANE")
            << "\",\"arrows\":[";
        for (size_t j = 0; j < lane.size(); ++j) {
            if (j) out << ',';
            out << '"' << directionName(kombinav::laneArrow(lane[j].shape)) << '"';
        }
        out << "],\"attributeType\":0,\"changeHint\":0,\"prevIndexes\":[],\"nextIndexes\":[]}";
    }
    out << "],\"markers\":[],\"isETC\":false}}]},";
}

inline std::string guidanceModels(const kombinav::NavView& nav, bool first) {
    const char* direction = maneuverDirection(nav.aa_type);
    double distance;
    if (!direction || !kombinav::distanceMetres(nav, distance) ||
        !(distance >= 0 && distance <= 100000)) return std::string();
    const bool lanes = validLanes(nav.lanes);

    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '[';
    if (first) {
        out << "{\"msg\":\"create\",\"data\":{\"appId\":\"BMW.HMI.Roadbook\","
            << "\"appName\":\"NBT-AAD Roadbook test\",\"appVendor\":\"Harman\","
            << "\"versionMajor\":3,\"versionMinor\":0,\"versionPatch\":0}},"
            << "{\"msg\":\"initialize\",\"data\":null},";
    }
    if (lanes) laneMessage(out, nav.lanes, distance);
    out << "{\"msg\":\"setDataMa\",\"data\":[";
    beginModel(out, "ManeuverDistance", kDistance);
    // RIC unit 3 is metres.
    out << ",\"distance\":" << distance << ",\"distStr\":\""
        << static_cast<int64_t>(distance + 0.5)
        << "\",\"distUnit\":3,\"routeEta\":0,\"barGraphVal\":-1,\"bgSegs\":[]}},";
    beginModel(out, "ManeuverState", kManeuverState);
    out << ",\"gsSwitches\":[10000,3000,180,70],\"popupLg\":"
        << (lanes ? "true" : "false") << ",\"popupLgId\":"
        << (lanes ? static_cast<unsigned>(kLanes) : 0xffffffffu) << "}},";
    beginModel(out, "ManeuverDescription", kManeuver);
    out << ",\"isPassive\":false,\"prevManeuverId\":4294967295,"
        << "\"nextManeuverId\":4294967295,\"prevManeuverJoined\":false,"
        << "\"nextManeuverJoined\":false,\"direction\":\"" << direction
        << "\",\"detailedManeuverId\":4294967295,\"manDistanceId\":" << kDistance
        << ",\"comingRoadId\":4294967295,\"maneuverStateId\":" << kManeuverState
        << ",\"trafficEvents\":[],\"laneGuidings\":[";
    if (lanes) out << kLanes;
    out << "],\"serviceAreas\":[]}}]},";
    listMessage(out, true);
    out << ',';
    controlMessage(out, "Active", true);
    out << ']';
    return out.str();
}

} // namespace roadbook
} // namespace nbt

#endif
