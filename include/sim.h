/*
Simple physics engine for a 2D environment
*/

// TODO: tomorrow: test FK with a simple 2-link arm, then add collision detection/conact point calculation
// after that: solve dynamics, and then integrate vel and position

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <unordered_set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <queue>
#include <cmath>
#include <limits>
#include <set>
#include <SFML/Graphics.hpp>

using json = nlohmann::json;
using Frame = std::tuple<double, double, double>;

namespace sim
{
    enum class ControlType
    {
        FORCETORQUE,
        POSITION
    };

    enum class LinkType
    {
        RECTANGLE,
        CIRCLE
    };

    enum class JointType
    {
        REVOLUTE,
        PRISMATIC,
        FIXED,
        FLOATING
    };

    enum class Material
    {
        PLASTIC,
        METAL
    };

    // later, need to be able to add objects. These will be unactuated and have no joints, but will be of the same types as links

    // TODO: in future, want to be able to manually specify links to ignore during collision checking (for example, two legs that need to swing by eachother)
    struct Link
    {
        std::string name;

        LinkType type;
        Material material;

        double d1; // for rectangle, this is width, for circle this is radius
        double d2;

        Frame frame; // current pos and orientation of link in world frame
        Frame vel; // current velocity of link in world frame

        Link(std::string name_, LinkType type_, Material material_, double d1_, double d2_)
            : name(name_), type(type_), material(material_)
        {
            if (type == LinkType::RECTANGLE)
            {
                d1 = d1_;
                d2 = d2_;
            }
            else if (type == LinkType::CIRCLE)
            {
                d1 = d1_;
                d2 = 0.0;
            }
            frame = {0.0, 0.0, 0.0};
        }
        Link(std::string name_, LinkType type_, Material material_, double d1_)
            : name(name_), type(type_), material(material_)
        {
            if (type == LinkType::RECTANGLE)
            {
                d1 = d1_;
                d2 = d1_; // make into a square
            }
            else if (type == LinkType::CIRCLE)
            {
                d1 = d1_;
                d2 = 0.0;
            }
            frame = {0.0, 0.0, 0.0};
        }
    };

    struct Joint
    {
        std::string name;
        Link *parent;
        Link *child;
        ControlType control_type;
        JointType joint_type;
        Frame offset;
        double limit_min;
        double limit_max;
        double kp;
        double kd;
        int axis; // for prismatic joints, 0 for x-axis, 1 for y-axis. Ignored for revolute joints

        double qpos = 0.0; // current position of joint
        double qvel = 0.0; // current velocity of joint
        double ctrl = 0.0; // current control input for joint

        Joint(std::string name_, Link *parent_, Link *child_, Frame offset_, ControlType control_type_, JointType joint_type_, double limit_min_, double limit_max_, double kp_, double kd_)
            : name(name_), parent(parent_), child(child_), offset(offset_), control_type(control_type_), joint_type(joint_type_), limit_min(limit_min_), limit_max(limit_max_), kp(kp_), kd(kd_)
        {
        }
        Joint(std::string name_, Link *parent_, Link *child_, Frame offset_, ControlType control_type_, JointType joint_type_, double limit_min_, double limit_max_)
            : name(name_), parent(parent_), child(child_), offset(offset_), control_type(control_type_), joint_type(joint_type_), limit_min(limit_min_), limit_max(limit_max_), kp(0.0), kd(0.0)
        {
        }
        Joint(std::string name_, Link *parent_, Link *child_, Frame offset_, ControlType control_type_, JointType joint_type_)
            : name(name_), parent(parent_), child(child_), offset(offset_), control_type(control_type_), joint_type(joint_type_), limit_min(0.0), limit_max(0.0), kp(0.0), kd(0.0)
        {
        }
    };

    struct Contact
    {
        // vectors in world frame
        Link *link_a;
        Link *link_b;
        double penetration_depth;                  // how far hapes have intersected along collision normal
        std::pair<double, double> contact_point;   // centroid of contact region in world frame. TODO: if this is unstable, can also track more than one contact point and use that.
        std::pair<double, double> contact_normal;  // axis along which there is collision (essentially the SAT axis with min distance), point from a to b
        std::pair<double, double> contact_tangent; // perpendicular to contact normal, for friction calcs
    };

    class Simulator
    {
    public:
        // render width and height: meters, render scale: how many pixels per meter
        Simulator(std::string scene_file, bool render, double render_width = 10.0, double render_height = 5.0, double render_scale = 100);
        ~Simulator();

        // PUBLIC API
        std::vector<Contact> check_collisions();
        // std::vector<Contact> check_collisions(std::vector<double> qpos); // arbitrary angles, could be useful for ppl doing collision checking stuff

        const int nu() const { return _nu; }
        void step(double dt); // updates sim

        void set_control(std::vector<double> ctrl);
        void reset(std::vector<double> qpos);
        void reset(std::vector<double> qpos, std::vector<Frame> base_link_pos);
        // void reset(std::vector<double> qpos, std::vector<double> qvel);

        // moving here for testing. Move back in end
        void _fk(); // compute forward kinematics to recalculate all link frames
        void _render_frame();

    private:
        std::unordered_map<Link *, std::vector<std::pair<Link *, Joint *>>> _tf_tree; // keeps track of basic tree structure in form of adjacency list
        std::vector<Link *> _links;
        std::vector<Joint *> _joints;
        std::set<std::pair<Link *, Link *>> _connected_links; // use set since can hash pairs. Keeps track of connected links so we avoid during collision checking

        Link *_world_link; // special link representing world frame
        std::vector<Link*> _base_links; // base links are all links connected to world through a floating joint. Can be more than one as there can be multiple robots
        int _nu;           // number of actuated joints

        std::unordered_map<int, Joint *> _joint_id_map; // map joint pointer to index such that we can index into qpos, qvel, ctrl

        // rendering
        bool _render;
        double _render_width;
        double _render_height;
        double _render_scale;
        sf::RenderWindow *_window;

        bool _rectangle_rectangle_collision_check(Link *link_a, Link *link_b, Contact &contact);
        bool _circle_circle_collision_check(Link *link_a, Link *link_b, Contact &contact);
        bool _rectangle_circle_collision_check(Link *link_a, Link *link_b, Contact &contact);
        bool _check_world_collision(Link *link, Contact &contact);

        std::pair<double, double> _transform_point(std::pair<double, double> point, Frame transform);

        inline LinkType _string_to_link_type(const std::string &type_str)
        {
            if (type_str == "rectangle" || type_str == "RECTANGLE")
                return LinkType::RECTANGLE;
            else if (type_str == "circle" || type_str == "CIRCLE")
                return LinkType::CIRCLE;
            else
                throw std::runtime_error("Invalid link type: " + type_str);
        }

        inline Material _string_to_material_type(const std::string &type_str)
        {
            if (type_str == "plastic" || type_str == "PLASTIC")
                return Material::PLASTIC;
            else if (type_str == "metal" || type_str == "METAL")
                return Material::METAL;
            else
                throw std::runtime_error("Invalid material type: " + type_str);
        }

        inline JointType _string_to_joint_type(const std::string &type_str)
        {
            if (type_str == "revolute" || type_str == "REVOLUTE")
                return JointType::REVOLUTE;
            else if (type_str == "prismatic" || type_str == "PRISMATIC")
                return JointType::PRISMATIC;
            else if (type_str == "fixed" || type_str == "FIXED")
                return JointType::FIXED;
            else if (type_str == "floating" || type_str == "FLOATING")
                return JointType::FLOATING;
            else
                throw std::runtime_error("Invalid joint type: " + type_str);
        }

        inline ControlType _string_to_control_type(const std::string &type_str)
        {
            if (type_str == "forcetorque" || type_str == "FORCETORQUE")
                return ControlType::FORCETORQUE;
            else if (type_str == "position" || type_str == "POSITION")
                return ControlType::POSITION;
            else
                throw std::runtime_error("Invalid control type: " + type_str);
        }
    };

}