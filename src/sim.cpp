#include "sim.h"

namespace sim
{
    // PUBLIC API
    Simulator::Simulator(std::string scene_file, bool render, double render_width, double render_height, double render_scale) : _render(render), _render_width(render_width), _render_height(render_height), _render_scale(render_scale)
    {
        std::ifstream f(scene_file);
        if (!f.is_open())
        {
            throw std::runtime_error("Could not open scene file: " + scene_file);
        }

        json data;
        try
        {
            f >> data;
        }
        catch (const json::parse_error &e)
        {
            throw std::runtime_error("JSON Parse Error: " + std::string(e.what()));
        }

        std::unordered_map<std::string, Link *> link_map;
        _world_link = new Link("world", LinkType::RECTANGLE, 0.0, 0.0, 0.0);
        _world_link->frame = {0.0, 0.0, 0.0};
        link_map["world"] = _world_link;

        std::unordered_set<Link *> child_links; // to identify the base link later

        try
        {
            if (data.contains("links") && data["links"].is_array())
            {
                for (const auto &link_json : data["links"])
                {
                    std::string name = link_json.at("name").get<std::string>();
                    LinkType type = _string_to_link_type(link_json.at("type"));
                    double mass = link_json.at("mass").get<double>();
                    double d1 = link_json.value("d1", 0.0);
                    double d2 = link_json.value("d2", 0.0);
                    Link *new_link = new Link(name, type, mass, d1, d2);
                    _links.push_back(new_link);
                    link_map[name] = new_link;
                }
            }

            if (data.contains("joints") && data["joints"].is_array())
            {
                for (const auto &joint_json : data["joints"])
                {
                    std::string name = joint_json.at("name").get<std::string>();

                    std::string parent_name = joint_json.at("parent").get<std::string>();
                    std::string child_name = joint_json.at("child").get<std::string>();
                    Frame offset = std::make_tuple(0.0, 0.0, 0.0);
                    if (joint_json.contains("offset") && joint_json["offset"].is_object())
                    {
                        const auto &offset_json = joint_json["offset"];

                        double ox = offset_json.at("x").get<double>();
                        double oy = offset_json.at("y").get<double>();
                        double oth = offset_json.at("theta").get<double>();

                        offset = {ox, oy, oth};
                    }

                    if ((parent_name != "world" && !link_map.count(parent_name)) ||
                        !link_map.count(child_name))
                    {
                        throw std::runtime_error("Joint '" + name + "' references a missing parent or child link.");
                    }

                    Link *parent_ptr = link_map[parent_name];
                    Link *child_ptr = link_map[child_name];

                    child_links.insert(child_ptr);

                    ControlType ctrl_type = _string_to_control_type(joint_json.at("control_type").get<std::string>());
                    JointType j_type = _string_to_joint_type(joint_json.at("joint_type").get<std::string>());

                    Joint *new_joint;

                    double limit_min = joint_json.at("limit_min").get<double>();
                    double limit_max = joint_json.at("limit_max").get<double>();

                    if (joint_json.contains("kp") && joint_json.contains("kd"))
                    {
                        double kp = joint_json.at("kp").get<double>();
                        double kd = joint_json.at("kd").get<double>();
                        new_joint = new Joint(
                            name, parent_ptr, child_ptr, offset, ctrl_type, j_type, limit_min, limit_max, kp, kd);
                    }
                    else
                    {
                        new_joint = new Joint(
                            name, parent_ptr, child_ptr, offset, ctrl_type, j_type, limit_min, limit_max);
                    }

                    if (j_type == JointType::PRISMATIC)
                    {
                        int axis = joint_json.at("axis").get<int>();
                        if (axis != 0 && axis != 1)
                        {
                            throw std::runtime_error("Invalid axis value for prismatic joint '" + name + "'. Must be 0 (x-axis) or 1 (y-axis).");
                        }
                        new_joint->axis = axis;
                    }

                    _joints.push_back(new_joint);

                    _tf_tree[parent_ptr].push_back({child_ptr, new_joint});

                    if (j_type != JointType::FIXED && j_type != JointType::FLOATING)
                    {
                        _joint_id_map[_nu] = new_joint; // since ctrl vector will only have actuated joints, we map to them
                        _nu++;
                    }
                    _connected_links.insert({parent_ptr, child_ptr});
                    _connected_links.insert({child_ptr, parent_ptr});
                }
            }

            // any link that is not a child link is a base link. Added as a floating joint to world link
            for (const auto &link : _links)
            {
                if (!child_links.count(link))
                {
                    Frame base_offset = std::make_tuple(0.0, 0.0, 0.0);
                    Joint *base_joint = new Joint(
                        link->name + "_base_joint", _world_link, link, base_offset, ControlType::POSITION, JointType::FLOATING);
                    _joints.push_back(base_joint);
                    _tf_tree[_world_link].push_back({link, base_joint});
                }
            }

            std::cout << "Model structure built successfully with " << _links.size() << " links, " << _joints.size() << " joints, and " << _nu << " actuated joint(s)." << std::endl;
            for (const auto &link : _links)
            {
                std::cout << "    Link: " << link->name << ", Type: " << (link->type == LinkType::RECTANGLE ? "Rectangle" : "Circle") << ", Mass: " << link->mass << std::endl;
            }
            for (const auto &joint : _joints)
            {
                std::cout << "    Joint: " << joint->name << ", Type: " << (joint->joint_type == JointType::REVOLUTE ? "Revolute" : (joint->joint_type == JointType::PRISMATIC ? "Prismatic" : (joint->joint_type == JointType::FIXED ? "Fixed" : "Floating"))) << ", Parent: " << joint->parent->name << ", Child: " << joint->child->name << ", Offset: (" << std::get<0>(joint->offset) << ", " << std::get<1>(joint->offset) << ", " << std::get<2>(joint->offset) << ")" << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Error building model structure: " + std::string(e.what()));
        }

        if (_render)
        {
            _window = new sf::RenderWindow(sf::VideoMode(_render_width * _render_scale, _render_height * _render_scale), "Simple Sim");
        }
    }

    std::vector<Contact> Simulator::check_collisions()
    {
        std::vector<Contact> contacts;
        for (size_t i_a = 0; i_a < _links.size(); i_a++)
        {
            for (size_t i_b = i_a + 1; i_b < _links.size(); i_b++)
            {
                Contact c;
                if (_connected_links.count({_links[i_a], _links[i_b]}))
                    continue;
                // if link a and link b have a joint, do not do collision checking
                if (_links[i_a]->type == LinkType::RECTANGLE && _links[i_b]->type == LinkType::RECTANGLE)
                {
                    if (_rectangle_rectangle_collision_check(_links[i_a], _links[i_b], c))
                        contacts.push_back(c);
                }
                else if (_links[i_a]->type == LinkType::CIRCLE && _links[i_b]->type == LinkType::CIRCLE)
                {
                    if (_circle_circle_collision_check(_links[i_a], _links[i_b], c))
                        contacts.push_back(c);
                }
                else
                {
                    if (_rectangle_circle_collision_check(_links[i_a], _links[i_b], c))
                        contacts.push_back(c);
                }
            }
        }

        for (auto link : _links)
        {
            Contact c;
            if (!_connected_links.count({link, _world_link}) && _check_world_collision(link, c))
                contacts.push_back(c);
        }
        // check for contacts with world link

        return contacts;
    }

    void Simulator::reset(std::vector<double> qpos)
    {
        if (qpos.size() != static_cast<size_t>(_nu))
        {
            throw std::invalid_argument("Invalid number of joint positions provided.");
        }
        for (size_t i = 0; i < qpos.size(); ++i)
        {
            Joint *joint = _joint_id_map[i];
            joint->qpos = qpos[i];
        }
        _fk();
    }

    Simulator::~Simulator()
    {
        for (auto link : _links)
        {
            delete link;
        }
        delete _world_link;
        for (auto joint : _joints)
        {
            delete joint;
        }
    }

    // PRIVATE API
    void Simulator::_fk()
    {
        std::queue<Link *> q;
        q.push(_world_link);
        // do a bfs and update link frames based on parent link frame and joint state
        while (!q.empty())
        {
            Link *parent_link = q.front();
            q.pop();
            auto [parent_x, parent_y, parent_theta] = parent_link->frame;
            for (const auto &[child_link, joint] : _tf_tree[parent_link])
            {
                auto [origin_x, origin_y, origin_theta] = joint->offset;

                if (joint->joint_type == JointType::REVOLUTE)
                {
                    double cos_p = std::cos(parent_theta);
                    double sin_p = std::sin(parent_theta);

                    // put pivot point in world frame (applying transformation matrix from world frame to parent frame)
                    auto [origin_world_x, origin_world_y] = _transform_point({origin_x, origin_y}, parent_link->frame);
                    double child_theta = parent_theta + origin_theta + joint->qpos;

                    // for rectangle, assumption that the joint is at the far left edge of the child link on the x-axis, and at the center
                    // of the link on the y-axis
                    // for circle, assumption that joint is at the center of the circle
                    double child_half_len = child_link->d1 / 2.0;
                    if (child_link->type == LinkType::CIRCLE)
                    {
                        child_half_len = 0;
                    }
                    double child_x = origin_world_x + std::cos(child_theta) * child_half_len;
                    double child_y = origin_world_y + std::sin(child_theta) * child_half_len;

                    child_link->frame = {child_x, child_y, child_theta};
                }
                else if (joint->joint_type == JointType::PRISMATIC)
                {
                    double cos_p = std::cos(parent_theta);
                    double sin_p = std::sin(parent_theta);

                    // put joint origin in world frame
                    auto [origin_world_x, origin_world_y] = _transform_point({origin_x, origin_y}, parent_link->frame);
                    double child_theta = parent_theta + origin_theta;

                    // compute axis direction in world frame
                    double axis_world_x = cos_p * (joint->axis == 0 ? 1.0 : 0.0) - sin_p * (joint->axis == 1 ? 1.0 : 0.0);
                    double axis_world_y = sin_p * (joint->axis == 0 ? 1.0 : 0.0) + cos_p * (joint->axis == 1 ? 1.0 : 0.0);

                    // compute how much to translate along world frame axis based on orientation and add to joint origin
                    double child_x = origin_world_x + axis_world_x * joint->qpos;
                    double child_y = origin_world_y + axis_world_y * joint->qpos;

                    // add offset to get to center of child link
                    double child_half_len = child_link->d1 / 2.0;
                    if (child_link->type == LinkType::CIRCLE)
                    {
                        child_half_len = 0;
                    }
                    child_x += std::cos(child_theta) * child_half_len;
                    child_y += std::sin(child_theta) * child_half_len;

                    // Save the computed center frame and queue the child
                    child_link->frame = {child_x, child_y, child_theta};
                }
                else if (joint->joint_type == JointType::FIXED)
                {
                    double cos_p = std::cos(parent_theta);
                    double sin_p = std::sin(parent_theta);

                    auto [origin_world_x, origin_world_y] = _transform_point({origin_x, origin_y}, parent_link->frame);

                    double child_theta = parent_theta + origin_theta;

                    double child_half_len = child_link->d1 / 2.0;
                    if (child_link->type == LinkType::CIRCLE)
                    {
                        child_half_len = 0;
                    }
                    double child_x = origin_world_x + std::cos(child_theta) * child_half_len;
                    double child_y = origin_world_y + std::sin(child_theta) * child_half_len;

                    child_link->frame = {child_x, child_y, child_theta};
                }
                else if (joint->joint_type == JointType::FLOATING)
                {
                    // if floating, then just leave transform as is and the pose is propogated down the tree
                    // for example, if base link is humanoid torso, then torso pose is set directly or through the prev timestep,
                    // and all links go from there
                }
                q.push(child_link);
            }
        }
    }

    std::pair<double, double> Simulator::_transform_point(std::pair<double, double> point, Frame transform)
    {
        auto [x, y] = point;
        auto [t_x, t_y, t_theta] = transform;

        double cos_t = std::cos(t_theta);
        double sin_t = std::sin(t_theta);

        double x_new = t_x + cos_t * x - sin_t * y;
        double y_new = t_y + sin_t * x + cos_t * y;

        return {x_new, y_new};
    }

    bool Simulator::_rectangle_rectangle_collision_check(Link *link_a, Link *link_b, Contact &contact)
    {
        if (link_a->type != LinkType::RECTANGLE || link_b->type != LinkType::RECTANGLE)
        {
            throw std::runtime_error("SAT currently only implemented for rectangle-rectangle collisions.");
        }

        // SAT refresh: if seperating axis exists, will be along one of the edges of the rectangles
        // since we are in 2D, only need to check 4 axes (2 will be redundant since they will be negated versions)
        // consisting of the x and y axes of each rectangles frame

        auto [a_x, a_y, a_theta] = link_a->frame;
        auto [b_x, b_y, b_theta] = link_b->frame;

        int a_width = link_a->d1;
        int a_height = link_a->d2;

        int b_width = link_b->d1;
        int b_height = link_b->d2;

        std::vector<std::pair<double, double>> corners_a =
            {
                {_transform_point({-a_width / 2.0, -a_height / 2.0}, link_a->frame)},
                {_transform_point({a_width / 2.0, -a_height / 2.0}, link_a->frame)},
                {_transform_point({a_width / 2.0, a_height / 2.0}, link_a->frame)},
                {_transform_point({-a_width / 2.0, a_height / 2.0}, link_a->frame)}};
        std::vector<std::pair<double, double>> corners_b =
            {
                {_transform_point({-b_width / 2.0, -b_height / 2.0}, link_b->frame)},
                {_transform_point({b_width / 2.0, -b_height / 2.0}, link_b->frame)},
                {_transform_point({b_width / 2.0, b_height / 2.0}, link_b->frame)},
                {_transform_point({-b_width / 2.0, b_height / 2.0}, link_b->frame)}};

        std::vector<std::pair<double, double>> axes =
            {
                {std::cos(a_theta), std::sin(a_theta)},  // x-axis of a
                {-std::sin(a_theta), std::cos(a_theta)}, // y-axis of a
                {std::cos(b_theta), std::sin(b_theta)},  // x-axis of b
                {-std::sin(b_theta), std::cos(b_theta)}  // y-axis of b
            };

        double min_penetration_depth = std::numeric_limits<double>::infinity();
        std::pair<double, double> best_contact_point = {0.0, 0.0};
        std::pair<double, double> best_normal = {0.0, 0.0};

        for (const auto &[ax, ay] : axes)
        {
            double min_a = std::numeric_limits<double>::infinity();
            double max_a = -std::numeric_limits<double>::infinity();
            size_t max_a_idx = 0;

            double min_b = std::numeric_limits<double>::infinity();
            double max_b = -std::numeric_limits<double>::infinity();
            size_t max_b_idx = 0;

            for (size_t i = 0; i < corners_a.size(); ++i)
            {
                double proj = corners_a[i].first * ax + corners_a[i].second * ay;
                if (proj < min_a)
                    min_a = proj;
                if (proj > max_a)
                {
                    max_a = proj;
                    max_a_idx = i;
                }
            }

            for (size_t i = 0; i < corners_b.size(); ++i)
            {
                double proj = corners_b[i].first * ax + corners_b[i].second * ay;
                if (proj < min_b)
                    min_b = proj;
                if (proj > max_b)
                {
                    max_b = proj;
                    max_b_idx = i;
                }
            }

            if (max_a < min_b || max_b < min_a)
            {
                return false;
            }

            double overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
            if (overlap < min_penetration_depth)
            {
                min_penetration_depth = overlap;
                best_normal = {ax, ay};

                // pick corner in overlap that is closest to the collision
                if ((max_a - min_b) < (max_b - min_a))
                {
                    // rectangle A's upper edge is pushing into B's lower edge.
                    // the penetrating vertex belongs to B, and it's the one closest to min_b
                    double min_dist = std::numeric_limits<double>::infinity();
                    for (const auto &corner : corners_b)
                    {
                        double proj = corner.first * ax + corner.second * ay;
                        if (proj < min_dist)
                        {
                            min_dist = proj;
                            best_contact_point = corner;
                        }
                    }
                }
                else
                {
                    // rectangle B's upper edge is pushing into A's lower edge.
                    // the penetrating vertex belongs to A, and it's the one closest to min_a
                    double min_dist = std::numeric_limits<double>::infinity();
                    for (const auto &corner : corners_a)
                    {
                        double proj = corner.first * ax + corner.second * ay;
                        if (proj < min_dist)
                        {
                            min_dist = proj;
                            best_contact_point = corner;
                        }
                    }
                }
            }
        }

        // project normal onto vector from a to b. If negative, negate so that
        // normal always points from a to b
        double vector_to_b_x = b_x - a_x;
        double vector_to_b_y = b_y - a_y;
        double dot_dir = best_normal.first * vector_to_b_x + best_normal.second * vector_to_b_y;

        if (dot_dir < 0)
        {
            best_normal.first = -best_normal.first;
            best_normal.second = -best_normal.second;
        }

        contact.link_a = link_a;
        contact.link_b = link_b;
        contact.penetration_depth = min_penetration_depth;
        contact.contact_normal = best_normal;
        contact.contact_tangent = {-best_normal.second, best_normal.first};
        contact.contact_point = best_contact_point;

        return true;
    }

    bool Simulator::_circle_circle_collision_check(Link *link_a, Link *link_b, Contact &contact)
    {
        if (link_a->type != LinkType::CIRCLE || link_b->type != LinkType::CIRCLE)
        {
            throw std::runtime_error("Circle-circle collision check can only be used for circle links.");
        }

        auto [a_x, a_y, _] = link_a->frame;
        auto [b_x, b_y, __] = link_b->frame;

        double radius_a = link_a->d1;
        double radius_b = link_b->d1;

        double dx = b_x - a_x;
        double dy = b_y - a_y;
        double dist_sq = dx * dx + dy * dy;
        double radius_sum = radius_a + radius_b;

        if (dist_sq >= radius_sum * radius_sum)
        {
            return false;
        }

        double dist = std::sqrt(dist_sq);
        double penetration_depth = radius_sum - dist;

        double normal_x, normal_y;

        // if very overlapped, pick normal direction unstable so just pick an arbitrary one
        if (dist < 1e-6)
        {
            normal_x = 1.0;
            normal_y = 0.0;
        }
        else
        {
            normal_x = dx / dist;
            normal_y = dy / dist;
        }

        contact.link_a = link_a;
        contact.link_b = link_b;
        contact.penetration_depth = penetration_depth;
        contact.contact_normal = {normal_x, normal_y};
        contact.contact_tangent = {-normal_y, normal_x};

        // solve for centroid of contact region
        double mid_x = a_x + normal_x * radius_a - normal_x * (penetration_depth * 0.5);
        double mid_y = a_y + normal_y * radius_a - normal_y * (penetration_depth * 0.5);
        contact.contact_point = {mid_x, mid_y};

        return true;
    }

    bool Simulator::_rectangle_circle_collision_check(Link *link_a, Link *link_b, Contact &contact)
    {
        if (link_a->type == LinkType::CIRCLE && link_b->type == LinkType::RECTANGLE)
        {
            bool hit = _rectangle_circle_collision_check(link_b, link_a, contact);
            if (hit)
            {
                contact.contact_normal.first = -contact.contact_normal.first;
                contact.contact_normal.second = -contact.contact_normal.second;
                contact.contact_tangent = {-contact.contact_normal.second, contact.contact_normal.first};
            }
            return hit;
        }
        else if (link_a->type != LinkType::RECTANGLE || link_b->type != LinkType::CIRCLE)
        {
            throw std::runtime_error("Rectangle-circle collision check can only be used for rectangle and circle links.");
        }

        auto [a_x, a_y, a_theta] = link_a->frame;
        auto [b_x, b_y, _] = link_b->frame;

        double a_width = link_a->d1;
        double a_height = link_a->d2;
        double b_radius = link_b->d1;

        // transform circle center into rectangles local frame
        double dx = b_x - a_x;
        double dy = b_y - a_y;
        double cos_a = std::cos(a_theta);
        double sin_a = std::sin(a_theta);

        double local_circle_x = cos_a * dx + sin_a * dy;
        double local_circle_y = -sin_a * dx + cos_a * dy;

        // find closest point on rectangle to circle center
        double half_w = a_width / 2.0;
        double half_h = a_height / 2.0;

        // clamped_x and clamped_y represent closest point to circle center
        double clamped_x = std::max(-half_w, std::min(half_w, local_circle_x));
        double clamped_y = std::max(-half_h, std::min(half_h, local_circle_y));

        double local_dist_x = local_circle_x - clamped_x;
        double local_dist_y = local_circle_y - clamped_y;
        double local_dist_sq = local_dist_x * local_dist_x + local_dist_y * local_dist_y;

        // if distance from circle center to closest point greater than rad, reject
        if (local_dist_sq >= b_radius * b_radius)
        {
            return false;
        }

        double local_dist = std::sqrt(local_dist_sq);
        double penetration_depth = b_radius - local_dist;

        double closest_world_x = a_x + (cos_a * clamped_x - sin_a * clamped_y);
        double closest_world_y = a_y + (sin_a * clamped_x + cos_a * clamped_y);

        double normal_x, normal_y;
        if (local_dist < 1e-6)
        {
            // the circle's center is deep inside the rectangle.
            // push out along the shallowest local face axis.
            double dlx = half_w - std::abs(local_circle_x);
            double dly = half_h - std::abs(local_circle_y);

            if (dlx < dly)
            {
                normal_x = (local_circle_x > 0) ? cos_a : -cos_a;
                normal_y = (local_circle_x > 0) ? sin_a : -sin_a;
                penetration_depth = dlx + b_radius;
            }
            else
            {
                normal_x = (local_circle_y > 0) ? -sin_a : sin_a;
                normal_y = (local_circle_y > 0) ? cos_a : -cos_a;
                penetration_depth = dly + b_radius;
            }
        }
        else
        {
            // normal points from closest surface feature straight to circle center
            double world_normal_x = b_x - closest_world_x;
            double world_normal_y = b_y - closest_world_y;
            double mag = std::sqrt(world_normal_x * world_normal_x + world_normal_y * world_normal_y);

            normal_x = world_normal_x / mag;
            normal_y = world_normal_y / mag;
        }

        contact.link_a = link_a;
        contact.link_b = link_b;
        contact.penetration_depth = penetration_depth;
        contact.contact_normal = {normal_x, normal_y};
        contact.contact_tangent = {-normal_y, normal_x};

        // midpoint selection: start at rectangle surface point and push
        // into the circle by half the penetration depth
        contact.contact_point = {
            closest_world_x + normal_x * (penetration_depth * 0.5),
            closest_world_y + normal_y * (penetration_depth * 0.5)};

        return true;
    }

    bool Simulator::_check_world_collision(Link *link, Contact &contact)
    {
        // calculate lowest point and see if it falls below y = 0
        auto [lx, ly, ltheta] = link->frame;

        double lowest_y = ly;
        std::pair<double, double> surface_point = {lx, ly};

        if (link->type == LinkType::CIRCLE)
        {
            double radius = link->d1;
            lowest_y = ly - radius;
            surface_point = {lx, lowest_y};
        }
        else if (link->type == LinkType::RECTANGLE)
        {
            // find lowest corner
            double w = link->d1;
            double h = link->d2;

            std::vector<std::pair<double, double>> corners = {
                _transform_point({-w / 2.0, -h / 2.0}, link->frame),
                _transform_point({w / 2.0, -h / 2.0}, link->frame),
                _transform_point({w / 2.0, h / 2.0}, link->frame),
                _transform_point({-w / 2.0, h / 2.0}, link->frame)};

            lowest_y = std::numeric_limits<double>::infinity();
            for (const auto &corner : corners)
            {
                if (corner.second < lowest_y)
                {
                    lowest_y = corner.second;
                    surface_point = corner;
                }
            }
        }

        if (lowest_y >= 0.0)
        {
            return false;
        }

        contact.link_a = _world_link; // World acts as the parent anchor
        contact.link_b = link;        // robot link is child
        contact.penetration_depth = 0.0 - lowest_y;

        // normal just points straight up always in world frame
        contact.contact_normal = {0.0, 1.0};
        contact.contact_tangent = {1.0, 0.0}; // just have tangent point along surface to the right

        // halfway into ground from contact point
        contact.contact_point = {surface_point.first, -contact.penetration_depth * 0.5};

        return true;
    }

    void Simulator::_render_frame()
    {
        if (!_render || !_window || !_window->isOpen())
            return;

        sf::Event event;
        while (_window->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                _window->close();
        }
        _window->clear(sf::Color::White);
        sf::View center_view;
        center_view.setSize(_render_width * _render_scale, _render_height * _render_scale);
        center_view.setCenter(0.0f, _render_height * _render_scale / 2.0); // force bottom center to be at (0, 0)
        _window->setView(center_view);

        for (const auto &link : _links)
        {
            if (link->type == LinkType::RECTANGLE)
            {
                sf::RectangleShape rect(sf::Vector2f(link->d1 * _render_scale, link->d2 * _render_scale));
                rect.setOrigin(link->d1 * _render_scale / 2.0, link->d2 * _render_scale / 2.0);
                float screen_x = std::get<0>(link->frame) * _render_scale;
                float screen_y = (_render_height - std::get<1>(link->frame)) * _render_scale;
                rect.setPosition(screen_x, screen_y);
                rect.setRotation(-std::get<2>(link->frame) * 180.0 / M_PI); // negate since SFML rotates CW but my convention is CCW is pos
                rect.setFillColor(sf::Color(230, 126, 34)); // Orange link body
                rect.setOutlineColor(sf::Color::Black);
                rect.setOutlineThickness(-2.f);
                _window->draw(rect);
            }
            else if (link->type == LinkType::CIRCLE)
            {
                sf::CircleShape circle(link->d1 * _render_scale);
                circle.setOrigin(link->d1 * _render_scale, link->d1 * _render_scale);
                float screen_x = std::get<0>(link->frame) * _render_scale;
                float screen_y = (_render_height - std::get<1>(link->frame)) * _render_scale;
                circle.setPosition(screen_x, screen_y);
                circle.setFillColor(sf::Color(230, 126, 34)); // Orange link body
                circle.setOutlineColor(sf::Color::Black);
                circle.setOutlineThickness(-2.f);
                _window->draw(circle);
            }
        }
        _window->display();
    }

}