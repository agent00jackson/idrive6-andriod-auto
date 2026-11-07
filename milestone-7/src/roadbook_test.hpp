// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#ifndef ROADBOOK_TEST_HPP
#define ROADBOOK_TEST_HPP

#include "kombi_nav_msg.hpp"

namespace nbt {
namespace roadbook {

// Runs on the existing navigation worker. The stock diagnostic POST backend
// takes over Roadbook until HmiMain3D restarts, so require a fresh manual arm.
class ParkedTest {
public:
    ParkedTest();
    bool start();
    bool armed() const;
    void step(const kombinav::NavView* nav, bool guidanceActive, bool sessionEnded, double now);
    void stop();
    void rejectArm();

private:
    bool preflight();
    bool enabled_;
    int port_;
    bool submitted_;
    double expires_;
    double nextUpdate_;
    unsigned long commits_;
    std::string lastModels_;
};

} // namespace roadbook
} // namespace nbt

#endif
