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
            reporter.writeValue("Force (body)", body_ptr->getWrench().force);
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

        getNextKinematicsNoCollison(dt, body, current, next);

        getNextKinematicsOnCollison(dt, body, current, next);
        //if (!getNextKinematicsOnCollison(dt, body, current, next, next_wrench))
        //    getNextKinematicsOnGround(dt, body, current, next, next_wrench);
        
        body.setKinematics(next);
        body.kinematicsUpdated();
    }

    static Wrench getBodyWrench(const PhysicsBody& body, const Quaternionr& orientation)
    {
        //set wrench sum to zero
        Wrench wrench = Wrench::zero();

        //calculate total force on rigid body's center of gravity
        for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
            //aggregate total
            const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
            const auto& vertex_wrench = vertex.getWrench();
            wrench += vertex_wrench;

            //add additional torque due to force applies farther than COG
            // tau = r X F
            wrench.torque += vertex.getPosition().cross(vertex_wrench.force);
        }

        //convert to world frame
        wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);
        wrench.torque = VectorMath::transformToWorldFrame(wrench.torque, orientation);

        return wrench;
    }

    static Wrench getDragWrench(const PhysicsBody& body, const Quaternionr& orientation, 
        const Twist& twist)
    {
        //add linear drag due to velocity we had since last dt seconds
        //drag vector magnitude is proportional to v^2, direction opposite of velocity
        //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
        //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
        //http://physics.stackexchange.com/questions/304742/angular-drag-on-body

        Wrench wrench = Wrench::zero();
        real_T air_density = body.getEnvironment().getState().air_density;

        for (uint vi = 0; vi < body.dragVertexCount(); ++vi) {
            const auto& vertex = body.getDragVertex(vi);
            Vector3r vel_vertex = twist.angular.cross(vertex.getPosition()) + twist.linear;
            Vector3r vel_body = VectorMath::transformToBodyFrame(vel_vertex, orientation);
            real_T vel_comp = vertex.getNormal().dot(vel_body);
            if (vel_comp > 0.1f) {
                Vector3r drag_force = vertex.getNormal() * (- vertex.getDragFactor() * air_density * vel_comp * vel_comp);
                Vector3r drag_torque = vertex.getPosition().cross(drag_force);

                wrench.force += drag_force;
                wrench.torque += drag_torque;
            }
        }

        //convert to world frame
        wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);
        wrench.torque = VectorMath::transformToWorldFrame(wrench.torque, orientation);

        return wrench;
    }

    static void updatePose(const Kinematics::State& current, Kinematics::State& next, real_T dt)
    {
        //get average during period
        Vector3r avg_velocity = (current.twist.linear + next.twist.linear) * 0.5f;
        Vector3r avg_angular = (current.twist.angular + next.twist.angular) * 0.5f;

        //update position
        next.pose.position = current.pose.position + (avg_velocity * dt);
        //update orientation
        next.pose.orientation = VectorMath::addAngularVelocity(current.pose.orientation, avg_angular, dt);
    }

    static void updateTwist(Kinematics::State& next, const Matrix3x3r& inertia_inv_world, const PhysicsBody& body)
    {
        next.twist.linear = next.momentums.linear * body.getMassInv();
        next.twist.angular = inertia_inv_world * next.momentums.angular;
    }

    void getNextKinematicsNoCollison(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next)
    {
        /************************* Get force and torque acting on body ************************/
        real_T dt_real = static_cast<real_T>(dt);

        //approximate current orientation by taking mid-point
        Quaternionr orientation_avg = VectorMath::addAngularVelocity(current.pose.orientation, current.twist.angular, dt_real * 0.5f);

        //get forces on the body
        Wrench body_wrench = getBodyWrench(body, orientation_avg);
        //TODO: should use midpoint for velocities below?
        Wrench drag_wrench = getDragWrench(body, orientation_avg, current.twist);
        next.wrench = body_wrench + drag_wrench;

        //integrate wrench during dt
        Vector3r force_avg = (body.getEnvironment().getState().gravity * body.getMass()) + ((current.wrench.force + next.wrench.force) * 0.5f);
        Vector3r torque_avg = (current.wrench.torque + next.wrench.torque) * 0.5f;

        //get world insertia tensor for current orientation
        Matrix3x3r rotation_avg = orientation_avg.toRotationMatrix();
        Matrix3x3r inertia_inv_world = rotation_avg * body.getInertiaInv() * rotation_avg.transpose();

        //update next moments and compute average for the period
        next.momentums.linear = current.momentums.linear + (force_avg * dt_real);
        next.momentums.angular = current.momentums.angular + (torque_avg * dt_real);

        updateTwist(next, inertia_inv_world, body);
        updatePose(current, next, dt_real);
    }

    bool getNextKinematicsOnCollison(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next)
    {
        real_T dt_real = static_cast<real_T>(dt);
        const CollisionInfo collison_info = body.getCollisionInfo();

        //if there is collison
        if (collison_info.has_collided) {
            //contact point vector
            Vector3r r = collison_info.impact_point - collison_info.position;

            //if we are hitting bottom of vehicle, we move contact vector in center (as multiple collison points are not supported)
            Vector3r body_z = VectorMath::transformToWorldFrame(Vector3r::UnitZ(), current.pose.orientation);
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
            Vector3r vnext_contact = next.twist.linear + next.twist.angular.cross(r);
            //is velocity at contact is in opposite direction as normal?
            real_T vnext_contact_normal_mag = collison_info.normal.dot(vnext_contact);

            //if we are not moving away
            if (Utils::isDefinitelyLessThan(vnext_contact_normal_mag, 0.0f)) {
                //we are going to reset next kinematics using collison response
                next.wrench = Wrench::zero();
                next.accelerations = Accelerations::zero();

                //get world inertia tensor for last state orientation (not the avg orientation)
                Matrix3x3r rotation = current.pose.orientation.toRotationMatrix();
                Matrix3x3r inertia_inv_world = rotation * body.getInertiaInv() * rotation.transpose();

                Vector3r rXn = r.cross(collison_info.normal);
                real_T j_denom = body.getMassInv() + (rXn.dot(inertia_inv_world * rXn));
                real_T j_nom = -(1 + body.getRestitution()) * vnext_contact_normal_mag;
                real_T j = j_nom / j_denom;

                next.momentums.linear = current.momentums.linear + (collison_info.normal * j);
                next.momentums.angular = current.momentums.angular + (r.cross(collison_info.normal) * j);

                updateTwist(next, inertia_inv_world, body);

                ////compute new velocity at contact
                //vnext_contact = next.twist.linear + next.twist.angular.cross(r);
                //Vector3r vtangent = vnext_contact - (collison_info.normal * (vnext_contact.dot(collison_info.normal)));
                //if (Utils::isDefinitelyGreaterThan(vtangent.squaredNorm(), 1E-6f)) {
                //    Vector3r vtangent_norm = vtangent.normalized();
                //    //contact velocity along the tangent
                //    Vector3r rXt = r.cross(vtangent_norm);
                //    real_T jt_denom = body.getMassInv() + (rXt.dot(inertia_inv_world * rXt));
                //    real_T jt_nom = vnext_contact.dot(vtangent_norm);   
                //    real_T jt = -jt_nom / jt_denom;
                //    real_T jt_clipped = Utils::clip(jt, -body.getFriction() * j, body.getFriction() * j);

                //    Vector3r linear_delta_friction = vtangent_norm * jt_clipped;
                //    Vector3r angular_delta_friction = r.cross(vtangent_norm) * jt_clipped;
                //    next.momentums.linear += linear_delta_friction;
                //    next.momentums.angular += angular_delta_friction;

                //    updateTwist(next, inertia_inv_world, body);

                //    Kinematics::State current_col = current;
                //    current_col.pose.position = collison_info.position + (collison_info.normal * collison_info.penetration_depth); // +next.twist.linear * (dt * kCollisionResponseCycles);

                //    updatePose(current_col, next, dt_real);
                //}
                //else
                    updatePose(current, next, dt_real);

                return true;
            }
            //else no collison response needed because we are already moving away
            //keep previously caomputed next kinematics
        }
        //else there was no collison

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


private:
    std::stringstream debug_string_;
    int grounded_;
};

}} //namespace
#endif
