#include "sim.h"

#include <iostream>
#include <tuple>

int main()
{
    sim::Simulator eng("scene.json");

    // Find torso and hip joint
    sim::Link* torso = nullptr;
    sim::Link* thigh = nullptr;
    sim::Joint* hip = nullptr;

    for (auto* link : eng.links())
    {
        if (link->name == "torso")
            torso = link;

        if (link->name == "thigh")
            thigh = link;
    }

    for (auto* joint : eng.joints())
    {
        if (joint->name == "hip_joint")
            hip = joint;
    }

    // Set floating-base/root pose manually
    torso->frame = {0.0, 0.5, 0.0};

    // Rotate thigh downward by 45 degrees
    hip->qpos = -0.785398; // -pi/4

    // Run FK
    eng._fk();

    auto [tx, ty, tt] = torso->frame;
    auto [hx, hy, ht] = thigh->frame;

    std::cout << "\n=== FK RESULT ===\n";

    std::cout << "Torso:\n";
    std::cout << "  x: " << tx
              << "  y: " << ty
              << "  theta: " << tt << "\n";

    std::cout << "Thigh:\n";
    std::cout << "  x: " << hx
              << "  y: " << hy
              << "  theta: " << ht << "\n";

    return 0;
}