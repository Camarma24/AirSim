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
        
        //init physics state
        auto initial_kinematics = Kinematics::State::zero();
        initial_kinematics.pose = Pose::zero();
        initial_kinematics.pose.position.z() = -1;
        initial_kinematics.twist.angular = Vector3r(0, 1, 0);
        msr::airlib::Environment::State initial_environment;
        initial_environment.position = initial_kinematics.pose.position;
        Environment environment(initial_environment);

        //set up physics body
        Matrix3x3r inertia = Matrix3x3r::Zero();
        inertia(0, 0) = 0.001f;
        inertia(1, 1) = 0.005f;
        inertia(2, 2) = 0.001f;
        
        DebugPhysicsBody body;
        body.initialize(2.0f, inertia, initial_kinematics, &environment);

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

            constexpr real_T body_height = -0.1f;
            constexpr real_T ground_level = -0.8f;
            const auto& pos = body.getKinematics().pose.position;
            if (pos.z() - body_height >= ground_level) {
                col.has_collided = true;
                col.impact_point = body.getKinematics().pose.position - Vector3r(0, body_height, body_height);
                col.normal = Vector3r(0, 0, -1);
                col.penetration_depth = 0;
                col.position = body.getKinematics().pose.position;

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