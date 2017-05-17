#pragma once

#include <memory>
#include "common/DebugClock.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/DebugPhysicsBody.hpp"


class StandAlonePhysics {
public:
    static void testCollison()
    {
        using namespace msr::airlib;

        std::shared_ptr<DebugClock> clock = std::make_shared<DebugClock>();
        ClockFactory::get(clock);

        constexpr real_T sphere_radius = 0.1f;
        constexpr real_T sphere_mass = 1.0f;

        //set up physics body
        Matrix3x3r inertia = Matrix3x3r::Zero();
        inertia(0, 0) = 2.0f/3 * sphere_mass * sphere_radius;
        inertia(1, 1) = 2.0f/3 * sphere_mass * sphere_radius;
        inertia(2, 2) = 2.0f/3 * sphere_mass * sphere_radius;

        //init physics state
        auto initial_kinematics = Kinematics::State::zero();
        initial_kinematics.pose = Pose::zero();
        initial_kinematics.pose.position.z() = -1;
        initial_kinematics.momentums.angular = Vector3r::Zero();//  inertia * Vector3r(0, 1, 0);
        msr::airlib::Environment::State initial_environment;
        initial_environment.position = initial_kinematics.pose.position;
        Environment environment(initial_environment);

        DebugPhysicsBody body;
        body.initialize(sphere_mass, inertia, initial_kinematics, &environment);

        //create physics engine
        FastPhysicsEngine physics;
        physics.insert(&body);


        //run
        unsigned int i = 0;
        while (true) {
            clock->stepNext();

            environment.update();
            body.update();
            physics.update();
            ++i;

            CollisionInfo col;

            constexpr real_T ground_level = -0.8f;
            const auto& pos = body.getKinematics().pose.position;
            real_T penetration = pos.z() + sphere_radius - ground_level;
            if (penetration >= 0) {
                col.has_collided = true;

                //desired params
                col.normal = Vector3r(0, 0, -1);
                Vector3r r = Vector3r(0, 0, sphere_radius);

                //computed params
                col.penetration_depth = penetration;
                col.impact_point = col.position + r;

                std::cout << "Col: " << VectorMath::toString(col.impact_point) << std::endl;
            }
            else {
                col.has_collided = false;
            }

            //col.has_collided = false;
            body.setCollisionInfo(col);
        }
    }
};