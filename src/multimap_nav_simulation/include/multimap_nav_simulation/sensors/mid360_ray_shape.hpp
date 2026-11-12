/*
Adapted into the MultiMapNav simulation package from livox_laser_simulation_ros2.
Local classes provide simulation-time acquisition without changing vendor sources.

MIT License

Copyright (c) 2023 Ricardo Casimiro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
//
// Created by lfc on 2021/2/28.
//

#ifndef MULTIMAP_NAV_SIMULATION__MID360_RAY_SHAPE_HPP_
#define MULTIMAP_NAV_SIMULATION__MID360_RAY_SHAPE_HPP_
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/util/system.hh>
#include <gazebo/ode/common.h>
#include <ignition/math6/ignition/math.hh>

namespace gazebo{
namespace physics{
class GZ_PHYSICS_VISIBLE MultimapMid360RayShape : public MultiRayShape{
    /// \brief Constructor.
    /// \param[in] _parent Parent Collision.
    public: explicit MultimapMid360RayShape(CollisionPtr _parent);

    /// \brief Destructor.
    public: virtual ~MultimapMid360RayShape();

    // Documentation inherited.
    public: virtual void UpdateRays();

    public: virtual void Init();

    public: std::vector<RayShapePtr> &RayShapes(){return rays;}
    /// \brief Ray-intersection callback.
    /// \param[in] _data Pointer to user data.
    /// \param[in] _o1 First geom to check for collisions.
    /// \param[in] _o2 Second geom to check for collisions.
    private: static void UpdateCallback(void *_data, dGeomID _o1,
                                        dGeomID _o2);

    /// \brief Add a ray to the collision.
    /// \param[in] _start Start of a ray.
    /// \param[in] _end End of a ray.
    public: void AddRay(const ignition::math::Vector3d &_start,
                           const ignition::math::Vector3d &_end);
    /// \brief Space to contain the ray space, for efficiency.
    private: dSpaceID superSpaceId;

    /// \brief Ray space for collision detector.
    private: dSpaceID raySpaceId;

 private:
    bool initializingBaseRanges = false;
    std::vector<RayShapePtr> livoxRays;
};
}
}


#endif  // MULTIMAP_NAV_SIMULATION__MID360_RAY_SHAPE_HPP_
