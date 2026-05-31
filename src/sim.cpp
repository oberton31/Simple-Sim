#include "sim.h"

/*
TODO tomorrow: seems to be working, but go through and add comments/documentation, ensuring
that I understand what this vibe coded things is doing. Also, clean up code if necessary

ALSO, SEEMS LIKE MATERIALS ARE TOO HEAVY, SOMETIMES THEY PIERCE FLOOR
*/

namespace sim
{
    // PUBLIC API
    Simulator::Simulator(std::string scene_file, bool render, double render_width, double render_height, double render_scale)
        : _render(render), _render_width(render_width), _render_height(render_height), _render_scale(render_scale)
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
        _world_link = new Link("world", LinkType::RECTANGLE, Material::PLASTIC, 0.0, 0.0);
        _world_link->frame = {0.0, 0.0, 0.0};
        link_map["world"] = _world_link;

        std::unordered_set<Link *> child_links;

        try
        {
            if (data.contains("links") && data["links"].is_array())
            {
                for (const auto &link_json : data["links"])
                {
                    std::string name = link_json.at("name").get<std::string>();
                    LinkType type = _string_to_link_type(link_json.at("type"));
                    Material material = _string_to_material_type(link_json.at("material"));
                    double d1 = link_json.value("d1", 0.0);
                    double d2 = link_json.value("d2", 0.0);
                    Link *new_link = new Link(name, type, material, d1, d2);
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
                        offset = {offset_json.at("x").get<double>(), offset_json.at("y").get<double>(), offset_json.at("theta").get<double>()};
                    }

                    if ((parent_name != "world" && !link_map.count(parent_name)) || !link_map.count(child_name))
                    {
                        throw std::runtime_error("Joint '" + name + "' references a missing parent or child link.");
                    }

                    Link *parent_ptr = link_map[parent_name];
                    Link *child_ptr = link_map[child_name];
                    child_links.insert(child_ptr);

                    ControlType ctrl_type = _string_to_control_type(joint_json.at("control_type").get<std::string>());
                    JointType j_type = _string_to_joint_type(joint_json.at("joint_type").get<std::string>());

                    double limit_min = joint_json.at("limit_min").get<double>();
                    double limit_max = joint_json.at("limit_max").get<double>();

                    Joint *new_joint;
                    if (joint_json.contains("kp") && joint_json.contains("kd"))
                    {
                        new_joint = new Joint(name, parent_ptr, child_ptr, offset, ctrl_type, j_type, limit_min, limit_max, joint_json.at("kp").get<double>(), joint_json.at("kd").get<double>());
                    }
                    else
                    {
                        if (ctrl_type == ControlType::POSITION)
                        {
                            throw std::invalid_argument("For position control, must specify kp and kd.");
                        }
                        new_joint = new Joint(name, parent_ptr, child_ptr, offset, ctrl_type, j_type, limit_min, limit_max);
                    }

                    if (j_type == JointType::PRISMATIC)
                    {
                        int axis = joint_json.at("axis").get<int>();
                        if (axis != 0 && axis != 1)
                        {
                            throw std::runtime_error("Invalid axis value for prismatic joint. Must be 0 or 1.");
                        }
                        new_joint->axis = axis;
                    }

                    _joints.push_back(new_joint);
                    _tf_tree[parent_ptr].push_back({child_ptr, new_joint});

                    if (j_type != JointType::FIXED && j_type != JointType::FLOATING)
                    {
                        _joint_id_map[_nu] = new_joint;
                        _nu++;
                    }

                    if (j_type == JointType::FLOATING)
                    {
                        _base_links.push_back(child_ptr);
                    }
                    else
                    {
                        _connected_links.insert({parent_ptr, child_ptr});
                        _connected_links.insert({child_ptr, parent_ptr});
                    }
                }
            }

            for (const auto &link : _links)
            {
                if (!child_links.count(link))
                {
                    Frame base_offset = std::make_tuple(0.0, 0.0, 0.0);
                    Joint *base_joint = new Joint(link->name + "_base_joint", _world_link, link, base_offset, ControlType::POSITION, JointType::FLOATING);
                    _joints.push_back(base_joint);
                    _tf_tree[_world_link].push_back({link, base_joint});
                    _base_links.push_back(link);
                }
            }
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Error building model structure: " + std::string(e.what()));
        }

        for (auto link : _links)
        {
            if (link->name == "world")
                continue;

            double density = (link->material == Material::METAL) ? 2.0 : 1.0;
            double area = (link->type == LinkType::RECTANGLE) ? (link->d1 * link->d2) : (M_PI * link->d1 * link->d1);
            link->mass = std::max(0.01, density * area);

            if (link->type == LinkType::RECTANGLE)
                link->inertia = (1.0 / 12.0) * link->mass * (link->d1 * link->d1 + link->d2 * link->d2);
            else
                link->inertia = 0.5 * link->mass * (link->d1 * link->d1);
        }

        if (_render)
        {
            _window = new sf::RenderWindow(sf::VideoMode(_render_width * _render_scale, _render_height * _render_scale), "Simple Sim");
            _window->setFramerateLimit(60);
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
            joint->qvel = 0.0;
        }
        _fk();
    }

    void Simulator::reset(std::vector<double> qpos, std::vector<Frame> base_link_pos)
    {
        if (qpos.size() != static_cast<size_t>(_nu))
        {
            throw std::invalid_argument("Invalid number of joint positions provided.");
        }
        if (_base_links.size() != base_link_pos.size())
        {
            throw std::invalid_argument("Invalid number of base link positions provided.");
        }
        for (size_t i = 0; i < _base_links.size(); i++)
        {
            _base_links[i]->frame = base_link_pos[i];
            _base_links[i]->vel = {0.0, 0.0, 0.0};
        }
        for (size_t i = 0; i < qpos.size(); i++)
        {
            Joint *joint = _joint_id_map[i];
            joint->qpos = qpos[i];
            joint->qvel = 0.0;
        }
        _fk();
    }

    void Simulator::set_control(std::vector<double> ctrl)
    {
        if (ctrl.size() != static_cast<size_t>(_nu))
        {
            throw std::invalid_argument("Invalid number of control inputs provided.");
        }
        for (size_t i = 0; i < ctrl.size(); i++)
        {
            Joint *joint = _joint_id_map[i];
            joint->ctrl = ctrl[i];
        }
    }

    void Simulator::step(double dt)
    {
        if (dt <= 0.0)
            return;

        _fk();

        std::vector<Contact> collisions = check_collisions();

        int base_state_size = static_cast<int>(_base_links.size()) * 3;
        int n_dof = base_state_size + _nu;

        std::vector<double> tau(n_dof, 0.0);
        std::vector<double> qvel_vec(n_dof, 0.0);

        // initialize qvel with current base link velocities
        for (size_t b = 0; b < _base_links.size(); ++b)
        {
            auto [vx, vy, omega] = _base_links[b]->vel;
            qvel_vec[b * 3 + 0] = vx;
            qvel_vec[b * 3 + 1] = vy;
            qvel_vec[b * 3 + 2] = omega;
        }
        for (int i = 0; i < _nu; ++i)
        {
            qvel_vec[base_state_size + i] = _joint_id_map[i]->qvel;
        }

        std::vector<std::vector<double>> M(n_dof, std::vector<double>(n_dof, 0.0));
        std::vector<double> G(n_dof, 0.0);
        const double gravity_constant = -9.81;

        for (auto link : _links)
        {
            if (link->name == "world")
                continue;

            std::vector<std::vector<double>> J_com(3, std::vector<double>(n_dof, 0.0));
            auto [cx, cy, ctheta] = link->frame;

            // navigate up tree to determine if the link is a descendant of the given base link
            auto is_descendant_of_base = [&](Link *base_link, Link *descendant)
            {
                Link *curr = descendant;
                while (curr && curr != _world_link)
                {
                    if (curr == base_link)
                        return true;
                    bool found_parent = false;
                    for (const auto &p_map : _tf_tree)
                    {
                        for (const auto &edge : p_map.second)
                        {
                            if (edge.first == curr)
                            {
                                curr = p_map.first;
                                found_parent = true;
                                break;
                            }
                        }
                        if (found_parent)
                            break;
                    }
                    if (!found_parent)
                        break;
                }
                return false;
            };

            // floating base contribution to Jacobian
            for (size_t b = 0; b < _base_links.size(); ++b)
            {
                if (!is_descendant_of_base(_base_links[b], link))
                    continue;

                auto [bx, by, bth] = _base_links[b]->frame;
                J_com[0][b * 3 + 0] = 1.0;
                J_com[1][b * 3 + 1] = 1.0;
                double rx = cx - bx;
                double ry = cy - by;
                J_com[0][b * 3 + 2] = -ry;
                J_com[1][b * 3 + 2] = rx;
                J_com[2][b * 3 + 2] = 1.0;
            }

            for (int i = 0; i < _nu; ++i)
            {
                Joint *joint = _joint_id_map[i];
                int target_dof_idx = base_state_size + i;

                // check if link is descendant of joint. If not, then joint does not affect link COM
                bool is_parent = false;
                Link *curr = link;
                while (curr && curr != _world_link)
                {
                    bool found_parent = false;
                    for (const auto &p_map : _tf_tree)
                    {
                        for (const auto &edge : p_map.second)
                        {
                            if (edge.first == curr)
                            {
                                if (edge.second == joint)
                                    is_parent = true;
                                curr = p_map.first;
                                found_parent = true;
                                break;
                            }
                        }
                        if (found_parent)
                            break;
                    }
                    if (!found_parent)
                        break;
                }

                if (is_parent)
                {
                    auto [p_x, p_y, p_th] = joint->parent->frame;
                    double parent_half_len = (joint->parent->type == LinkType::CIRCLE || joint->parent->name == "world") ? 0.0 : joint->parent->d1 / 2.0;

                    double jx = p_x + std::cos(p_th) * parent_half_len;
                    double jy = p_y + std::sin(p_th) * parent_half_len;

                    auto [ox, oy, oth] = joint->offset;
                    double cos_p = std::cos(p_th);
                    double sin_p = std::sin(p_th);
                    jx += cos_p * ox - sin_p * oy;
                    jy += sin_p * ox + cos_p * oy;

                    // jacobian contribution from this joint
                    if (joint->joint_type == JointType::REVOLUTE)
                    {
                        J_com[0][target_dof_idx] = -(cy - jy);
                        J_com[1][target_dof_idx] = (cx - jx);
                        J_com[2][target_dof_idx] = 1.0;
                    }
                    else if (joint->joint_type == JointType::PRISMATIC)
                    {
                        double ax = std::cos(p_th) * (joint->axis == 0 ? 1.0 : 0.0) - std::sin(p_th) * (joint->axis == 1 ? 1.0 : 0.0);
                        double ay = std::sin(p_th) * (joint->axis == 0 ? 1.0 : 0.0) + std::cos(p_th) * (joint->axis == 1 ? 1.0 : 0.0);
                        J_com[0][target_dof_idx] = ax;
                        J_com[1][target_dof_idx] = ay;
                        J_com[2][target_dof_idx] = 0.0;
                    }
                }
            }

            for (int r = 0; r < n_dof; ++r)
            {
                for (int c = 0; c < n_dof; ++c)
                {
                    M[r][c] += link->mass * (J_com[0][r] * J_com[0][c] + J_com[1][r] * J_com[1][c]) + link->inertia * (J_com[2][r] * J_com[2][c]);
                }
                G[r] += link->mass * gravity_constant * J_com[1][r];
            }
        }

        // compute control torques
        for (int i = 0; i < _nu; ++i)
        {
            Joint *joint = _joint_id_map[i];
            int tau_idx = base_state_size + i;

            if (joint->control_type == ControlType::FORCETORQUE)
            {
                tau[tau_idx] = joint->ctrl;
            }
            else
            {
                double e = joint->ctrl - joint->qpos;
                double de = 0.0 - joint->qvel;
                double tau_pd = joint->kp * e + joint->kd * de;
                tau[tau_idx] = tau_pd - G[tau_idx];
            }
        }

        // process each collision manifold to resolve velocities via impulses before forward dynamics
        for (const auto &c : collisions)
        {
            std::vector<double> J_normal(n_dof, 0.0);
            std::vector<double> J_tangent(n_dof, 0.0);

            // if joint is ancestor the link with contact, construct jacobian contribution for that joint
            for (int j = 0; j < n_dof; ++j)
            {
                double jx = 0.0, jy = 0.0;
                bool active = false;

                if (j < base_state_size)
                {
                    std::tie(jx, jy, std::ignore) = _base_links[j / 3]->frame;
                    active = true;
                }
                else
                {
                    Joint *joint = _joint_id_map[j - base_state_size];
                    Link *curr = c.link_b;
                    while (curr && curr != _world_link)
                    {
                        bool found = false;
                        for (const auto &p_map : _tf_tree)
                        {
                            for (const auto &edge : p_map.second)
                            {
                                if (edge.first == curr)
                                {
                                    if (edge.second == joint)
                                        active = true;
                                    curr = p_map.first;
                                    found = true;
                                    break;
                                }
                            }
                            if (found)
                                break;
                        }
                        if (!found)
                            break;
                    }

                    if (active)
                    {
                        auto [p_x, p_y, p_th] = joint->parent->frame;
                        double parent_half_len = (joint->parent->type == LinkType::CIRCLE || joint->parent->name == "world") ? 0.0 : joint->parent->d1 / 2.0;
                        jx = p_x + std::cos(p_th) * parent_half_len;
                        jy = p_y + std::sin(p_th) * parent_half_len;

                        auto [ox, oy, oth] = joint->offset;
                        double cos_p = std::cos(p_th);
                        double sin_p = std::sin(p_th);
                        jx += cos_p * ox - sin_p * oy;
                        jy += sin_p * ox + cos_p * oy;
                    }
                }

                if (active)
                {
                    double rx = c.contact_point.first - jx;
                    double ry = c.contact_point.second - jy;

                    if (j < base_state_size)
                    {
                        int component = j % 3;
                        if (component == 0)
                        {
                            J_normal[j] = c.contact_normal.first;
                            J_tangent[j] = c.contact_tangent.first;
                        }
                        else if (component == 1)
                        {
                            J_normal[j] = c.contact_normal.second;
                            J_tangent[j] = c.contact_tangent.second;
                        }
                        else
                        {
                            J_normal[j] = (rx * c.contact_normal.second - ry * c.contact_normal.first);
                            J_tangent[j] = (rx * c.contact_tangent.second - ry * c.contact_tangent.first);
                        }
                    }
                    else
                    {
                        Joint *joint = _joint_id_map[j - base_state_size];
                        if (joint->joint_type == JointType::REVOLUTE)
                        {
                            J_normal[j] = (rx * c.contact_normal.second - ry * c.contact_normal.first);
                            J_tangent[j] = (rx * c.contact_tangent.second - ry * c.contact_tangent.first);
                        }
                        else if (joint->joint_type == JointType::PRISMATIC)
                        {
                            double jth = std::get<2>(joint->parent->frame);
                            double ax = std::cos(jth) * (joint->axis == 0 ? 1.0 : 0.0) - std::sin(jth) * (joint->axis == 1 ? 1.0 : 0.0);
                            double ay = std::sin(jth) * (joint->axis == 0 ? 1.0 : 0.0) + std::cos(jth) * (joint->axis == 1 ? 1.0 : 0.0);
                            J_normal[j] = (c.contact_normal.first * ax + c.contact_normal.second * ay);
                            J_tangent[j] = (c.contact_tangent.first * ax + c.contact_tangent.second * ay);
                        }
                    }
                }
            }

            // compute contact point velocity along normal and tangent directions
            double point_normal_vel = 0.0;
            double point_tangent_vel = 0.0;
            for (int j = 0; j < n_dof; ++j)
            {
                if (j < base_state_size)
                {
                    point_normal_vel += J_normal[j] * qvel_vec[j];
                    point_tangent_vel += J_tangent[j] * qvel_vec[j];
                }
                else
                {
                    double j_qvel = _joint_id_map[j - base_state_size]->qvel;
                    point_normal_vel += J_normal[j] * j_qvel;
                    point_tangent_vel += J_tangent[j] * j_qvel;
                }
            }

            // apply normal impulse if penetration
            if (point_normal_vel < 0.0)
            {
                std::vector<std::vector<double>> M_temp = M;
                for (int k = 0; k < n_dof; ++k)
                    M_temp[k][k] += 1e-6;

                std::vector<double> M_inv_JT = J_normal;
                for (int i = 0; i < n_dof; ++i)
                {
                    for (int j = i + 1; j < n_dof; ++j)
                    {
                        double factor = M_temp[j][i] / M_temp[i][i];
                        for (int k = i; k < n_dof; ++k)
                            M_temp[j][k] -= factor * M_temp[i][k];
                        M_inv_JT[j] -= factor * M_inv_JT[i];
                    }
                }
                for (int i = n_dof - 1; i >= 0; --i)
                {
                    double sum = 0.0;
                    for (int j = i + 1; j < n_dof; ++j)
                        sum += M_temp[i][j] * M_inv_JT[j];
                    M_inv_JT[i] = (M_inv_JT[i] - sum) / M_temp[i][i];
                }

                double effective_normal_inertia = 0.0;
                for (int j = 0; j < n_dof; ++j)
                    effective_normal_inertia += J_normal[j] * M_inv_JT[j];

                if (effective_normal_inertia > 1e-6)
                {
                    double impulse_mag = -point_normal_vel / effective_normal_inertia;

                    for (int j = 0; j < n_dof; ++j)
                    {
                        qvel_vec[j] += M_inv_JT[j] * impulse_mag;
                    }

                    for (size_t b = 0; b < _base_links.size(); ++b)
                    {
                        _base_links[b]->vel = {
                            qvel_vec[b * 3 + 0],
                            qvel_vec[b * 3 + 1],
                            qvel_vec[b * 3 + 2]};
                    }
                    for (int i = 0; i < _nu; ++i)
                    {
                        _joint_id_map[i]->qvel = qvel_vec[base_state_size + i];
                    }

                    // apply tangential impulse (i.e. friction) if there is velocity along tangent direction
                    if (std::abs(point_tangent_vel) > 1e-8)
                    {
                        std::vector<std::vector<double>> M_tangent = M;
                        for (int k = 0; k < n_dof; ++k)
                            M_tangent[k][k] += 1e-6;

                        std::vector<double> M_inv_JT_t = J_tangent;
                        for (int i = 0; i < n_dof; ++i)
                        {
                            for (int j = i + 1; j < n_dof; ++j)
                            {
                                double factor = M_tangent[j][i] / M_tangent[i][i];
                                for (int k = i; k < n_dof; ++k)
                                    M_tangent[j][k] -= factor * M_tangent[i][k];
                                M_inv_JT_t[j] -= factor * M_inv_JT_t[i];
                            }
                        }
                        for (int i = n_dof - 1; i >= 0; --i)
                        {
                            double sum = 0.0;
                            for (int j = i + 1; j < n_dof; ++j)
                                sum += M_tangent[i][j] * M_inv_JT_t[j];
                            M_inv_JT_t[i] = (M_inv_JT_t[i] - sum) / M_tangent[i][i];
                        }

                        double effective_tangent_inertia = 0.0;
                        for (int j = 0; j < n_dof; ++j)
                            effective_tangent_inertia += J_tangent[j] * M_inv_JT_t[j];

                        if (effective_tangent_inertia > 1e-6)
                        {
                            double tangent_impulse = -point_tangent_vel / effective_tangent_inertia;

                            const double mu_friction = 0.4;
                            double friction_limit = mu_friction * impulse_mag;

                            if (tangent_impulse > friction_limit)
                                tangent_impulse = friction_limit;
                            if (tangent_impulse < -friction_limit)
                                tangent_impulse = -friction_limit;

                            if (std::abs(tangent_impulse) > 1e-12)
                            {
                                for (int j = 0; j < n_dof; ++j)
                                {
                                    qvel_vec[j] += M_inv_JT_t[j] * tangent_impulse;
                                }
                                for (size_t b = 0; b < _base_links.size(); ++b)
                                {
                                    _base_links[b]->vel = {
                                        qvel_vec[b * 3 + 0],
                                        qvel_vec[b * 3 + 1],
                                        qvel_vec[b * 3 + 2]};
                                }
                                for (int i = 0; i < _nu; ++i)
                                {
                                    _joint_id_map[i]->qvel = qvel_vec[base_state_size + i];
                                }
                            }
                        }
                    }
                }
            }
        }

        // now, solving M * qacc = tau + G for qacc (equation is classic rigid body dynamics with contact forces)

        // right hand side of equation
        std::vector<double> RHS(n_dof, 0.0);
        for (int i = 0; i < n_dof; ++i)
        {
            RHS[i] = tau[i] + G[i];
        }

        std::vector<std::vector<double>> M_solved = M;
        for (int i = 0; i < n_dof; ++i)
            M_solved[i][i] += 1e-6;

        std::vector<double> qacc(n_dof, 0.0);
        for (int i = 0; i < n_dof; ++i)
        {
            for (int j = i + 1; j < n_dof; ++j)
            {
                double factor = M_solved[j][i] / M_solved[i][i];
                for (int k = i; k < n_dof; ++k)
                    M_solved[j][k] -= factor * M_solved[i][k];
                RHS[j] -= factor * RHS[i];
            }
        }

        // solve for acceleerations
        for (int i = n_dof - 1; i >= 0; --i)
        {
            double sum = 0.0;
            for (int j = i + 1; j < n_dof; ++j)
                sum += M_solved[i][j] * qacc[j];
            qacc[i] = (RHS[i] - sum) / M_solved[i][i];
        }

        // integrate forward to get new velocities and positions
        for (size_t b = 0; b < _base_links.size(); ++b)
        {
            auto [bx, by, bth] = _base_links[b]->frame;
            double vx = qvel_vec[b * 3 + 0] + qacc[b * 3 + 0] * dt;
            double vy = qvel_vec[b * 3 + 1] + qacc[b * 3 + 1] * dt;
            double omega = qvel_vec[b * 3 + 2] + qacc[b * 3 + 2] * dt;

            _base_links[b]->vel = {vx, vy, omega};
            _base_links[b]->frame = {bx + vx * dt, by + vy * dt, bth + omega * dt};
        }

        // apply joint limits
        for (int i = 0; i < _nu; ++i)
        {
            Joint *joint = _joint_id_map[i];
            joint->qvel += qacc[base_state_size + i] * dt;
            joint->qpos += joint->qvel * dt;

            if (joint->qpos > joint->limit_max)
            {
                joint->qpos = joint->limit_max;
                if (joint->qvel > 0.0)
                    joint->qvel = 0.0;
            }
            if (joint->qpos < joint->limit_min)
            {
                joint->qpos = joint->limit_min;
                if (joint->qvel < 0.0)
                    joint->qvel = 0.0;
            }
        }

        // update forward kinematics
        _fk();

        // numerical drift may cause penetrations. To resolve this, we do a post-update check.
        // where we use positional correction to resolve penetrations (Baumgarte-style stabilization)
        std::vector<Contact> post_collisions = check_collisions();

        const double positional_allowance = 0.6; // how much over overlap to resolve each step
        const double contact_slop = 0.002;

        // accumulation buffers ensure that multi-point contacts (e.g., flat box landings)
        // blend their corrections together concurrently instead of overwriting each other.
        std::vector<std::pair<double, double>> base_pos_deltas(_base_links.size(), {0.0, 0.0});
        std::vector<double> base_theta_deltas(_base_links.size(), 0.0);
        std::vector<double> joint_pos_deltas(_nu, 0.0);

        for (const auto &c : post_collisions)
        {
            if (c.penetration_depth <= contact_slop)
                continue;

            std::vector<double> J_normal(n_dof, 0.0);

            for (int j = 0; j < n_dof; ++j)
            {
                double jx = 0.0, jy = 0.0;
                bool active = false;

                if (j < base_state_size)
                {
                    std::tie(jx, jy, std::ignore) = _base_links[j / 3]->frame;
                    active = true;
                }
                else
                {
                    Joint *joint = _joint_id_map[j - base_state_size];
                    Link *curr = c.link_b;
                    while (curr && curr != _world_link)
                    {
                        bool found = false;
                        for (const auto &p_map : _tf_tree)
                        {
                            for (const auto &edge : p_map.second)
                            {
                                if (edge.first == curr)
                                {
                                    if (edge.second == joint)
                                        active = true;
                                    curr = p_map.first;
                                    found = true;
                                    break;
                                }
                            }
                            if (found)
                                break;
                        }
                        if (!found)
                            break;
                    }

                    if (active)
                    {
                        auto [p_x, p_y, p_th] = joint->parent->frame;
                        double parent_half_len = (joint->parent->type == LinkType::CIRCLE || joint->parent->name == "world") ? 0.0 : joint->parent->d1 / 2.0;
                        jx = p_x + std::cos(p_th) * parent_half_len;
                        jy = p_y + std::sin(p_th) * parent_half_len;

                        auto [ox, oy, oth] = joint->offset;
                        double cos_p = std::cos(p_th);
                        double sin_p = std::sin(p_th);
                        jx += cos_p * ox - sin_p * oy;
                        jy += sin_p * ox + cos_p * oy;
                    }
                }

                if (active)
                {
                    double rx = c.contact_point.first - jx;
                    double ry = c.contact_point.second - jy;

                    if (j < base_state_size)
                    {
                        int component = j % 3;
                        if (component == 0)
                            J_normal[j] = c.contact_normal.first;
                        else if (component == 1)
                            J_normal[j] = c.contact_normal.second;
                        else
                            J_normal[j] = (rx * c.contact_normal.second - ry * c.contact_normal.first);
                    }
                    else
                    {
                        Joint *joint = _joint_id_map[j - base_state_size];
                        if (joint->joint_type == JointType::REVOLUTE)
                        {
                            J_normal[j] = (rx * c.contact_normal.second - ry * c.contact_normal.first);
                        }
                        else if (joint->joint_type == JointType::PRISMATIC)
                        {
                            double jth = std::get<2>(joint->parent->frame);
                            double ax = std::cos(jth) * (joint->axis == 0 ? 1.0 : 0.0) - std::sin(jth) * (joint->axis == 1 ? 1.0 : 0.0);
                            double ay = std::sin(jth) * (joint->axis == 0 ? 1.0 : 0.0) + std::cos(jth) * (joint->axis == 1 ? 1.0 : 0.0);
                            J_normal[j] = (c.contact_normal.first * ax + c.contact_normal.second * ay);
                        }
                    }
                }
            }

            std::vector<std::vector<double>> M_temp = M;
            for (int k = 0; k < n_dof; ++k)
                M_temp[k][k] += M_temp[k][k] * 1e-6; // Relative scaling blocks artificial softness!

            std::vector<double> M_inv_JT = J_normal;
            for (int i = 0; i < n_dof; ++i)
            {
                for (int j = i + 1; j < n_dof; ++j)
                {
                    double factor = M_temp[j][i] / M_temp[i][i];
                    for (int k = i; k < n_dof; ++k)
                        M_temp[j][k] -= factor * M_temp[i][k];
                    M_inv_JT[j] -= factor * M_inv_JT[i];
                }
            }
            for (int i = n_dof - 1; i >= 0; --i)
            {
                double sum = 0.0;
                for (int j = i + 1; j < n_dof; ++j)
                    sum += M_temp[i][j] * M_inv_JT[j];
                M_inv_JT[i] = (M_inv_JT[i] - sum) / M_temp[i][i];
            }

            double effective_normal_inertia = 0.0;
            for (int j = 0; j < n_dof; ++j)
                effective_normal_inertia += J_normal[j] * M_inv_JT[j];

            if (effective_normal_inertia < 1e-6)
                continue;

            double correction_mag = ((c.penetration_depth - contact_slop) * positional_allowance) / effective_normal_inertia;

            for (size_t b = 0; b < _base_links.size(); ++b)
            {
                base_pos_deltas[b].first += M_inv_JT[b * 3 + 0] * correction_mag;
                base_pos_deltas[b].second += M_inv_JT[b * 3 + 1] * correction_mag;
                base_theta_deltas[b] += M_inv_JT[b * 3 + 2] * correction_mag;
            }
            for (int i = 0; i < _nu; ++i)
            {
                joint_pos_deltas[i] += M_inv_JT[base_state_size + i] * correction_mag;
            }
        }

        for (size_t b = 0; b < _base_links.size(); ++b)
        {
            auto [bx, by, bth] = _base_links[b]->frame;
            _base_links[b]->frame = {
                bx + base_pos_deltas[b].first,
                by + base_pos_deltas[b].second,
                bth + base_theta_deltas[b]};
        }
        for (int i = 0; i < _nu; ++i)
        {
            _joint_id_map[i]->qpos += joint_pos_deltas[i];
        }

        _fk();

        if (_render)
        {
            _window->clear(sf::Color::White);
            _render_frame();
        }
    }

    Simulator::~Simulator()
    {
        for (auto link : _links)
            delete link;
        delete _world_link;
        for (auto joint : _joints)
            delete joint;
        if (_render && _window)
            delete _window;
    }

    // PRIVATE API

    void Simulator::_fk()
    {
        // BFS through tree to compute forward kinematics
        std::queue<Link *> q;
        q.push(_world_link);
        _world_link->vel = {0.0, 0.0, 0.0};

        while (!q.empty())
        {
            Link *parent_link = q.front();
            q.pop();

            auto [parent_x, parent_y, parent_theta] = parent_link->frame;
            auto [parent_vx, parent_vy, parent_omega] = parent_link->vel;

            for (const auto &[child_link, joint] : _tf_tree[parent_link])
            {
                auto [origin_x, origin_y, origin_theta] = joint->offset;

                if (joint->joint_type == JointType::REVOLUTE)
                {
                    auto [origin_world_x, origin_world_y] = _transform_point({origin_x, origin_y}, parent_link->frame);
                    double child_theta = parent_theta + origin_theta + joint->qpos;

                    double child_half_len = (child_link->type == LinkType::CIRCLE) ? 0.0 : child_link->d1 / 2.0;
                    double child_x = origin_world_x + std::cos(child_theta) * child_half_len;
                    double child_y = origin_world_y + std::sin(child_theta) * child_half_len;
                    child_link->frame = {child_x, child_y, child_theta};

                    double r_px = origin_world_x - parent_x;
                    double r_py = origin_world_y - parent_y;
                    double v_pivot_x = parent_vx - parent_omega * r_py;
                    double v_pivot_y = parent_vy + parent_omega * r_px;

                    double child_omega = parent_omega + joint->qvel;
                    double r_cx = child_x - origin_world_x;
                    double r_cy = child_y - origin_world_y;
                    double child_vx = v_pivot_x - child_omega * r_cy;
                    double child_vy = v_pivot_y + child_omega * r_cx;
                    child_link->vel = {child_vx, child_vy, child_omega};
                }
                else if (joint->joint_type == JointType::PRISMATIC)
                {
                    auto [origin_world_x, origin_world_y] = _transform_point({origin_x, origin_y}, parent_link->frame);
                    double child_theta = parent_theta + origin_theta;

                    double cos_p = std::cos(parent_theta);
                    double sin_p = std::sin(parent_theta);
                    double axis_world_x = cos_p * (joint->axis == 0 ? 1.0 : 0.0) - sin_p * (joint->axis == 1 ? 1.0 : 0.0);
                    double axis_world_y = sin_p * (joint->axis == 0 ? 1.0 : 0.0) + cos_p * (joint->axis == 1 ? 1.0 : 0.0);

                    double child_x = origin_world_x + axis_world_x * joint->qpos;
                    double child_y = origin_world_y + axis_world_y * joint->qpos;

                    double child_half_len = (child_link->type == LinkType::CIRCLE) ? 0.0 : child_link->d1 / 2.0;
                    child_x += std::cos(child_theta) * child_half_len;
                    child_y += std::sin(child_theta) * child_half_len;
                    child_link->frame = {child_x, child_y, child_theta};

                    double r_px = origin_world_x - parent_x;
                    double r_py = origin_world_y - parent_y;
                    double v_pivot_base_x = parent_vx - parent_omega * r_py;
                    double v_pivot_base_y = parent_vy + parent_omega * r_px;

                    double child_vx = v_pivot_base_x + axis_world_x * joint->qvel;
                    double child_vy = v_pivot_base_y + axis_world_y * joint->qvel;
                    double child_omega = parent_omega;

                    double r_cx = child_x - origin_world_x;
                    double r_cy = child_y - origin_world_y;
                    child_vx -= child_omega * r_cy;
                    child_vy += child_omega * r_cx;
                    child_link->vel = {child_vx, child_vy, child_omega};
                }
                else if (joint->joint_type == JointType::FIXED)
                {
                    auto [origin_world_x, origin_world_y] = _transform_point({origin_x, origin_y}, parent_link->frame);
                    double child_theta = parent_theta + origin_theta;

                    double child_half_len = (child_link->type == LinkType::CIRCLE) ? 0.0 : child_link->d1 / 2.0;
                    double child_x = origin_world_x + std::cos(child_theta) * child_half_len;
                    double child_y = origin_world_y + std::sin(child_theta) * child_half_len;
                    child_link->frame = {child_x, child_y, child_theta};

                    double r_cx = child_x - parent_x;
                    double r_cy = child_y - parent_y;
                    double child_vx = parent_vx - parent_omega * r_cy;
                    double child_vy = parent_vy + parent_omega * r_cx;
                    child_link->vel = {child_vx, child_vy, parent_omega};
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
        auto [a_x, a_y, a_theta] = link_a->frame;
        auto [b_x, b_y, b_theta] = link_b->frame;

        double a_width = link_a->d1;
        double a_height = link_a->d2;
        double b_width = link_b->d1;
        double b_height = link_b->d2;

        Frame a_center_frame = {a_x, a_y, a_theta};
        Frame b_center_frame = {b_x, b_y, b_theta};

        std::vector<std::pair<double, double>> corners_a = {
            _transform_point({-a_width / 2.0, -a_height / 2.0}, a_center_frame),
            _transform_point({a_width / 2.0, -a_height / 2.0}, a_center_frame),
            _transform_point({a_width / 2.0, a_height / 2.0}, a_center_frame),
            _transform_point({-a_width / 2.0, a_height / 2.0}, a_center_frame)};
        std::vector<std::pair<double, double>> corners_b = {
            _transform_point({-b_width / 2.0, -b_height / 2.0}, b_center_frame),
            _transform_point({b_width / 2.0, -b_height / 2.0}, b_center_frame),
            _transform_point({b_width / 2.0, b_height / 2.0}, b_center_frame),
            _transform_point({-b_width / 2.0, b_height / 2.0}, b_center_frame)};

        std::vector<std::pair<double, double>> axes = {
            {std::cos(a_theta), std::sin(a_theta)},
            {-std::sin(a_theta), std::cos(a_theta)},
            {std::cos(b_theta), std::sin(b_theta)},
            {-std::sin(b_theta), std::cos(b_theta)}};

        double min_penetration_depth = std::numeric_limits<double>::infinity();
        std::pair<double, double> best_contact_point = {0.0, 0.0};
        std::pair<double, double> best_normal = {0.0, 0.0};

        for (const auto &[ax, ay] : axes)
        {
            double min_a = std::numeric_limits<double>::infinity(), max_a = -std::numeric_limits<double>::infinity();
            double min_b = std::numeric_limits<double>::infinity(), max_b = -std::numeric_limits<double>::infinity();

            for (const auto &c : corners_a)
            {
                double proj = c.first * ax + c.second * ay;
                min_a = std::min(min_a, proj);
                max_a = std::max(max_a, proj);
            }
            for (const auto &c : corners_b)
            {
                double proj = c.first * ax + c.second * ay;
                min_b = std::min(min_b, proj);
                max_b = std::max(max_b, proj);
            }

            if (max_a < min_b || max_b < min_a)
                return false;

            double overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
            if (overlap < min_penetration_depth)
            {
                min_penetration_depth = overlap;
                best_normal = {ax, ay};

                if ((max_a - min_b) < (max_b - min_a))
                {
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

        double vector_to_b_x = std::get<0>(b_center_frame) - std::get<0>(a_center_frame);
        double vector_to_b_y = std::get<1>(b_center_frame) - std::get<1>(a_center_frame);
        if ((best_normal.first * vector_to_b_x + best_normal.second * vector_to_b_y) < 0)
        {
            best_normal.first = -best_normal.first;
            best_normal.second = -best_normal.second;
        }

        contact.link_a = link_a;
        contact.link_b = link_b;
        contact.penetration_depth = min_penetration_depth;
        contact.contact_normal = {best_normal.first, best_normal.second};
        contact.contact_tangent = {-best_normal.second, best_normal.first};
        contact.contact_point = best_contact_point;
        return true;
    }

    bool Simulator::_circle_circle_collision_check(Link *link_a, Link *link_b, Contact &contact)
    {
        auto [a_x, a_y, _] = link_a->frame;
        auto [b_x, b_y, __] = link_b->frame;

        double radius_a = link_a->d1;
        double radius_b = link_b->d1;

        double dx = b_x - a_x;
        double dy = b_y - a_y;
        double dist_sq = dx * dx + dy * dy;
        double radius_sum = radius_a + radius_b;

        if (dist_sq >= radius_sum * radius_sum)
            return false;

        double dist = std::sqrt(dist_sq);
        double penetration_depth = radius_sum - dist;

        double normal_x = (dist < 1e-6) ? 1.0 : dx / dist;
        double normal_y = (dist < 1e-6) ? 0.0 : dy / dist;

        contact.link_a = link_a;
        contact.link_b = link_b;
        contact.penetration_depth = penetration_depth;
        contact.contact_normal = {normal_x, normal_y};
        contact.contact_tangent = {-normal_y, normal_x};
        contact.contact_point = {a_x + normal_x * (radius_a - penetration_depth * 0.5), a_y + normal_y * (radius_a - penetration_depth * 0.5)};
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

        auto [a_x, a_y, a_theta] = link_a->frame;
        auto [b_x, b_y, _] = link_b->frame;

        double a_width = link_a->d1;
        double a_height = link_a->d2;
        double b_radius = link_b->d1;

        Frame a_center_frame = {a_x, a_y, a_theta};

        double dx = b_x - std::get<0>(a_center_frame);
        double dy = b_y - std::get<1>(a_center_frame);
        double cos_a = std::cos(a_theta);
        double sin_a = std::sin(a_theta);

        double local_circle_x = cos_a * dx + sin_a * dy;
        double local_circle_y = -sin_a * dx + cos_a * dy;

        double half_w = a_width / 2.0;
        double half_h = a_height / 2.0;

        double clamped_x = std::max(-half_w, std::min(half_w, local_circle_x));
        double clamped_y = std::max(-half_h, std::min(half_h, local_circle_y));

        double local_dist_x = local_circle_x - clamped_x;
        double local_dist_y = local_circle_y - clamped_y;
        double local_dist_sq = local_dist_x * local_dist_x + local_dist_y * local_dist_y;

        if (local_dist_sq >= b_radius * b_radius)
            return false;

        double local_dist = std::sqrt(local_dist_sq);
        double penetration_depth = b_radius - local_dist;

        double closest_world_x = std::get<0>(a_center_frame) + (cos_a * clamped_x - sin_a * clamped_y);
        double closest_world_y = std::get<1>(a_center_frame) + (sin_a * clamped_x + cos_a * clamped_y);

        double normal_x, normal_y;
        if (local_dist < 1e-6)
        {
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
        contact.contact_point = {closest_world_x + normal_x * (penetration_depth * 0.5), closest_world_y + normal_y * (penetration_depth * 0.5)};

        return true;
    }

    bool Simulator::_check_world_collision(Link *link, Contact &contact)
    {
        auto [lx, ly, ltheta] = link->frame;
        double lowest_y = ly;
        std::pair<double, double> surface_point = {lx, ly};

        if (link->type == LinkType::CIRCLE)
        {
            lowest_y = ly - link->d1;
            surface_point = {lx, lowest_y};
        }
        else if (link->type == LinkType::RECTANGLE)
        {
            double w = link->d1;
            double h = link->d2;

            Frame center_frame = {lx, ly, ltheta};

            std::vector<std::pair<double, double>> corners = {
                _transform_point({-w / 2.0, -h / 2.0}, center_frame),
                _transform_point({w / 2.0, -h / 2.0}, center_frame),
                _transform_point({w / 2.0, h / 2.0}, center_frame),
                _transform_point({-w / 2.0, h / 2.0}, center_frame)};

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
            return false;

        contact.link_a = _world_link;
        contact.link_b = link;
        contact.penetration_depth = 0.0 - lowest_y;
        contact.contact_normal = {0.0, 1.0};
        contact.contact_tangent = {1.0, 0.0};
        contact.contact_point = {surface_point.first, 0.0};
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
            {
                _window->close();
                return;
            }
        }
        _window->clear(sf::Color::White);
        sf::View center_view;
        center_view.setSize(_render_width * _render_scale, _render_height * _render_scale);
        center_view.setCenter(0.0f, _render_height * _render_scale / 2.0);
        _window->setView(center_view);

        for (const auto &link : _links)
        {
            if (link->name == "world")
                continue;

            if (link->type == LinkType::RECTANGLE)
            {
                sf::RectangleShape rect(sf::Vector2f(link->d1 * _render_scale, link->d2 * _render_scale));
                rect.setOrigin(link->d1 * _render_scale / 2.0, link->d2 * _render_scale / 2.0);
                float screen_x = std::get<0>(link->frame) * _render_scale;
                float screen_y = (_render_height - std::get<1>(link->frame)) * _render_scale;
                rect.setPosition(screen_x, screen_y);
                rect.setRotation(-std::get<2>(link->frame) * 180.0 / M_PI);
                rect.setFillColor(sf::Color(230, 126, 34));
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
                circle.setFillColor(sf::Color(230, 126, 34));
                circle.setOutlineColor(sf::Color::Black);
                circle.setOutlineThickness(-2.f);
                _window->draw(circle);
            }
        }
        _window->display();
    }
}
