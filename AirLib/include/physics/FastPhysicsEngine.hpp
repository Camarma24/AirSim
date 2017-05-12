// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_FastPhysicsEngine_hpp
#define airsim_core_FastPhysicsEngine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {

class FastPhysicsEngine : public PhysicsEngineBase {
public:
    FastPhysicsEngine()
    { 
        FastPhysicsEngine::reset();
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        for (PhysicsBody* body_ptr : *this) {
            initPhysicsBody(body_ptr);
        }
    }

    virtual void insert(PhysicsBody* body_ptr) override
    {
        PhysicsEngineBase::insert(body_ptr);

        initPhysicsBody(body_ptr);
    }

    virtual void update() override
    {
        for (PhysicsBody* body_ptr : *this) {
            updatePhysics(*body_ptr);
        }
    }
    virtual void reportState(StateReporter& reporter) override
    {
        for (PhysicsBody* body_ptr : *this) {
            reporter.writeValue("Phys", debug_string_.str());
            reporter.writeValue("Is Gounded", grounded_);
            reporter.writeValue("Force (world)", body_ptr->getWrench().force);
            reporter.writeValue("Torque (body)", body_ptr->getWrench().torque);
        }
        //call base
        UpdatableObject::reportState(reporter);
    }
    //*** End: UpdatableState implementation ***//

private:
    void initPhysicsBody(PhysicsBody* body_ptr)
    {
        body_ptr->last_kinematics_time = clock()->nowNanos();
    }

    void updatePhysics(PhysicsBody& body)
    {
        TTimeDelta dt = clock()->updateSince(body.last_kinematics_time);

        //get current kinematics state of the body - this state existed since last dt seconds
        const Kinematics::State& current = body.getKinematics();
        Kinematics::State next;
        Wrench next_wrench;

        getNextKinematicsNoCollison(dt, body, current, next, next_wrench);

        getNextKinematicsOnCollison(dt, body, current, next, next_wrench);
        //if (!getNextKinematicsOnCollison(dt, body, current, next, next_wrench))
        //    getNextKinematicsOnGround(dt, body, current, next, next_wrench);
        
        body.setKinematics(next);
        body.setWrench(next_wrench);
        body.kinematicsUpdated();
    }

    bool getNextKinematicsOnCollison(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench)
    {
        static constexpr uint kCollisionResponseCycles = 1;

        /************************* Collison response ************************/
        real_T dt_real = static_cast<real_T>(dt);
        const CollisionInfo collison_info = body.getCollisionInfo();
        //if there is collison
        if (collison_info.has_collided) {
            //determine if we are going away from collison?
            Vector3r vnext_linear = next.twist.linear + next.accelerations.linear * dt_real;
            Vector3r vnext_angular = next.twist.angular + next.accelerations.angular * dt_real;

            //contact point vector
            Vector3r r = collison_info.impact_point - collison_info.position;

            //are we hitting the bottom?
            Vector3r body_z = VectorMath::transformToWorldFrame(Vector3r::UnitZ(), current.pose.orientation, true);
            real_T body_z_world_z = body_z.dot(Vector3r::UnitZ()) - 1;
            if (body_z_world_z <= 0.1f && body_z_world_z >= -0.1f) {
                real_T impact_z_dir = collison_info.normal.dot(Vector3r::UnitZ()) + 1;
                if (impact_z_dir <= 0.1f && impact_z_dir >= -0.1f) {
                    if (r.z() >= 0.16f && r.z() <= 0.20f) {
                        r.x() = 0;
                        r.y() = 0;
                    }
                }
            }


            //net velocity at contact point
            Vector3r vnext_angular_world = VectorMath::transformToWorldFrame(vnext_angular, next.pose.orientation, true);
            Vector3r vnext_contact = vnext_linear + vnext_angular_world.cross(r);
            //is velocity at contact is in opposite direction as normal?
            real_T vnext_contact_normal_mag = collison_info.normal.dot(vnext_contact);

            //if we are not moving away
            if (Utils::isDefinitelyLessThan(vnext_contact_normal_mag, 0.0f)) {
                //we are going to reset next kinematics using collison response
                //first we convert our input vectors to body coordinates

                Vector3r vb = VectorMath::transformToBodyFrame(current.twist.linear, next.pose.orientation);
                Vector3r rb = VectorMath::transformToBodyFrame(r, current.pose.orientation);
                Vector3r nb = VectorMath::transformToBodyFrame(collison_info.normal, current.pose.orientation);
                Vector3r vcb = vb + current.twist.angular.cross(rb);

                //compute the magnitude of impulse force assuming collison is with fixed body
                Vector3r rbXnb = rb.cross(nb);
                //Vector3r rbXnbI = body.getInertiaInv() * rbXnb;

                real_T j_nom = -(1 + body.getRestitution()) * vcb.dot(nb);
                //real_T j_denom = 1.0f / body.getMass() + (body.getInertiaInv() * r.cross(collison_info.normal).cross(r)).dot(collison_info.normal);
                //real_T rbXnb_dot_rbXnbI = rbXnb.dot(rbXnbI);
                Vector3r i_temp = body.getInertiaInv() * rbXnb;
                Vector3r i_temp2 = i_temp.cross(rb);
                real_T j_denom = (1.0f / body.getMass()) + i_temp2.dot(nb);
                    //(body.getInertiaInv() * r.cross(collison_info.normal).cross(r)).dot(collison_info.normal);
                real_T j = j_nom / j_denom;

                next.twist.linear = current.twist.linear + (j * collison_info.normal / body.getMass());
                Vector3r a_temp = rb.cross(j * nb);
                next.twist.angular = current.twist.angular + (body.getInertiaInv() * a_temp);

                ////compute new velocity at contact
                //vb = VectorMath::transformToBodyFrame(current.twist.linear, next.pose.orientation);
                //vcb = vb + current.twist.angular.cross(rb);
                //Vector3r vtb = vcb - nb * vcb.dot(nb);
                //if (Utils::isDefinitelyGreaterThan(vtb.squaredNorm(), 1E-6f)) {
                //    Vector3r vtnb = vtb.normalized();
                //    real_T jf_nom = - vcb.dot(vtnb);
                //    //real_T jf_denom = 1.0f / body.getMass() + (body.getInertiaInv() * r.cross(tangent).cross(r)).dot(tangent);
                //    real_T jf_denom = 1.0f / body.getMass() +
                //        rb.cross(vtnb).dot(body.getInertiaInv() * rb.cross(vtnb));
                //    real_T jf = Utils::clip(jf_nom / jf_denom, -body.getFriction() * j, body.getFriction() * j);

                //    Vector3r vtn = VectorMath::transformToWorldFrame(vtnb, current.pose.orientation);
                //    next.twist.linear += (jf * vtn / body.getMass());
                //    next.twist.angular += (jf * body.getInertiaInv() * rb.cross(vtnb));
                //}

                //there is no acceleration during collison response
                next.accelerations.linear = Vector3r::Zero();
                next.accelerations.angular = Vector3r::Zero();

                //if there is penetration, move out
                next.pose.position = collison_info.position + (collison_info.normal * collison_info.penetration_depth); // +next.twist.linear * (dt * kCollisionResponseCycles);
                next_wrench = Wrench::zero();

                return true;
            }
            //else no collison response needed because we are already moving away
            //keep previously caomputed next kinematics
        }

        return false;
    }

    bool getNextKinematicsOnGround(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench)
    {
        /************************* reset state if we have hit the ground ************************/
        real_T min_z_over_ground = body.getEnvironment().getState().min_z_over_ground;
        grounded_ = 0;
        if (min_z_over_ground <= next.pose.position.z()) {
            grounded_ = 1;
            next.pose.position.z() = min_z_over_ground;

            real_T z_proj = static_cast<real_T>(next.twist.linear.z() + next.accelerations.linear.z() * dt);
            if (Utils::isDefinitelyLessThan(0.0f, z_proj)) {
                grounded_ = 2;
                next.twist = Twist::zero();
                next.accelerations.linear = Vector3r::Zero();
                next.accelerations.angular = Vector3r::Zero();
                //reset roll/pitch - px4 seems to have issue with this
                real_T r, p, y;
                VectorMath::toEulerianAngle(current.pose.orientation, p, r, y);
                next.pose.orientation = VectorMath::toQuaternion(0, 0, y);

                next_wrench = Wrench::zero();
            }
        }

        return grounded_ != 0;
    }

    void getNextKinematicsNoCollison(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench)
    {
        /************************* Get force and torque acting on body ************************/
        real_T dt_real = static_cast<real_T>(dt);

        //set wrench sum to zero
        Wrench wrench = Wrench::zero();

        Vector3r cog = Vector3r::Zero();
        const CollisionInfo collison_info = body.getCollisionInfo();
        //if there is collison we will apply force around contact point to generate torque
        if (collison_info.has_collided) {
            cog = VectorMath::transformToBodyFrame(collison_info.impact_point - collison_info.position, current.pose.orientation, true);
        }

        //calculate total force on rigid body's center of gravity
        for (uint i = 0; i < body.vertexCount(); ++i) {
            //aggregate total
            const PhysicsBodyVertex& vertex = body.getVertex(i);
            const auto& vertex_wrench = vertex.getWrench();
            wrench += vertex_wrench;

            //add additional torque due to force applies farther than COG
            // tau = r X F
            wrench.torque +=  (vertex.getPosition() - cog).cross(vertex_wrench.force);
        }

        //transoform force to world frame
        Vector3r force_world_generated = VectorMath::transformToWorldFrame(wrench.force, current.pose.orientation, true);

        //add linear drag due to velocity we had since last dt seconds
        //drag vector magnitude is proportional to v^2, direction opposite of velocity
        //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
        //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
        Vector3r avg_velocity = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
        Vector3r avg_velocity_body = VectorMath::transformToBodyFrame(avg_velocity, current.pose.orientation, true);
        real_T avg_velocity_body_norm = avg_velocity_body.norm();
        Vector3r drag_force_world = Vector3r::Zero();
        if (!Utils::isApproximatelyZero(avg_velocity_body_norm, 1E-1f)) {
            Vector3r drag_force_body = 
                body.getLinearDragFactor()
                .cwiseProduct(avg_velocity_body)
                .cwiseProduct(avg_velocity_body);
            drag_force_world = -avg_velocity / avg_velocity_body_norm * drag_force_body.norm();
        }
        Vector3r force_net_world = force_world_generated + drag_force_world;

        //similarly calculate angular drag
        //note that angular velocity, acceleration, torque are already in body frame
        //http://physics.stackexchange.com/questions/304742/angular-drag-on-body
        Vector3r avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);
        real_T avg_angular_norm = avg_angular.norm();
        Vector3r angular_drag = Vector3r::Zero();
        //if angular velocity is too low (for example, random noise), 1/norm can get randomly big and generate huge drag
        if (!Utils::isApproximatelyZero(avg_angular_norm, 1E-1f)) {
            Vector3r angular_drag_mag = body.getAngularDragFactor()
                .cwiseProduct(avg_angular)
                .cwiseProduct(avg_angular);
            angular_drag = -avg_angular / avg_angular_norm * angular_drag_mag.norm();
        }
        Vector3r torque_net = wrench.torque + angular_drag;


        /************************* Update accelerations due to force and torque ************************/
        //get new acceleration due to force - we'll use this acceleration in next time step
        next.accelerations.linear = (force_net_world / body.getMass()) + body.getEnvironment().getState().gravity;

        //get new angular acceleration
        //Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
        //we will use torque to find out the angular acceleration
        //angular momentum L = I * omega
        Vector3r angular_momentum = body.getInertia() * avg_angular;
        Vector3r angular_momentum_rate = torque_net - avg_angular.cross(angular_momentum);
        //new angular acceleration - we'll use this acceleration in next time step
        next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;



        /************************* Update pose and twist after dt ************************/
        //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
        next.pose.position = current.pose.position + avg_velocity * dt_real;
        next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);

        //use angular velocty in body frame to calculate angular displacement in last dt seconds
        real_T angle_per_unit = avg_angular.norm();
        if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f)) {
            //convert change in angle to unit quaternion
            AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
            Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
            /*
                Add change in angle to previous orientation.
                Proof that this is q0 * q1:
                If rotated vector is qx*v*qx' then qx is attitude
                Initially we have q0*v*q0'
                Lets transform this to body coordinates to get
                q0'*(q0*v*q0')*q0
                Then apply q1 rotation on it to get
                q1(q0'*(q0*v*q0')*q0)q1'
                Then transform back to world coordinate
                q0(q1(q0'*(q0*v*q0')*q0)q1')q0'
                which simplifies to
                q0(q1(v)q1')q0'
                Thus new attitude is q0q1
            */
            next.pose.orientation = current.pose.orientation * angle_dt_q;
            if (VectorMath::hasNan(next.pose.orientation))
                Utils::DebugBreak();

            //re-normalize quaternion to avoid accumulating error
            next.pose.orientation.normalize();
        } 
        else //no change in angle, because angular velocity is zero (normalized vector is undefined)
            next.pose.orientation = current.pose.orientation;

        next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);

        next_wrench = Wrench(force_net_world, torque_net);
    }

private:
    std::stringstream debug_string_;
    int grounded_;
};

}} //namespace
#endif
