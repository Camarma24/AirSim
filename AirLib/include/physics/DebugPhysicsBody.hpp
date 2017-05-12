// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_DebugPhysicsBody_hpp
#define airsim_core_DebugPhysicsBody_hpp

#include "PhysicsBody.hpp"
#include <exception>
#include <iostream>

namespace msr { namespace airlib {

class DebugPhysicsBody : public PhysicsBody {
public: 
    DebugPhysicsBody()
    {
        vertex_.initialize(Vector3r(0, 0, 0), Vector3r(0, 0, -1));
    }

    virtual Vector3r getLinearDragFactor() const override 
    {
        return linear_drag_factor_;
    }
    virtual Vector3r getAngularDragFactor() const override 
    {
        return linear_drag_factor_;
    }
    virtual uint vertexCount() const override
    {
        return 1;
    }
    virtual void kinematicsUpdated() override 
    {
        const auto& kinematics = this->getKinematics();
        std::cout << " Pos: " << VectorMath::toString(kinematics.pose.position);
        std::cout << " Ori: " << VectorMath::toString(kinematics.pose.orientation) << std::endl;
        std::cout << " Lin Vel: " << VectorMath::toString(kinematics.twist.linear);
        std::cout << " Ang Vel: " << VectorMath::toString(kinematics.twist.angular) << std::endl;
    }
    virtual real_T getRestitution() const override 
    {
        return 0.5f;
    }
    virtual real_T getFriction() const override
    {
        return 0.7f;
    }
    virtual PhysicsBodyVertex& getVertex(uint index) override
    {
        return vertex_;
    }
    virtual const PhysicsBodyVertex& getVertex(uint index) const override
    {
        return vertex_;
    }

private:
    Vector3r linear_drag_factor_ = Vector3r(0, 0, 0);

    class Vertex : public PhysicsBodyVertex {
    public:
        virtual void setWrench(Wrench& wrench) override
        {
            //wrench.torque = Vector3r(0, 0.01f, 0);
        }
    } vertex_;
};

}} //namespace
#endif
