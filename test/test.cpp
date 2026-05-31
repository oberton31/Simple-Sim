#include "sim.h"

#include <iostream>
#include <tuple>

int main()
{
    sim::Simulator eng("scene.json", true, 10, 5, 100);
    eng.reset({M_PI / 2, 0, 0});
    // eng.reset({0, 0}, {{0, 0.5, 0}});
    eng.set_control({M_PI / 4, 0, 0});
    auto c = eng.check_collisions();
    for (const auto &contact : c)
    {
        std::cout << "Collision between " << contact.link_a->name << " and " << contact.link_b->name << std::endl;
        std::cout << "Penetration depth: " << contact.penetration_depth << std::endl;
        std::cout << "Contact point: (" << contact.contact_point.first << ", " << contact.contact_point.second << ")" << std::endl;
        std::cout << "Contact normal: (" << contact.contact_normal.first << ", " << contact.contact_normal.second << ")" << std::endl;
        std::cout << "Contact tangent: (" << contact.contact_tangent.first << ", " << contact.contact_tangent.second << ")" << std::endl;
    }
    while (true)
    {
        eng.step(0.001);
    }

    return 0;
}