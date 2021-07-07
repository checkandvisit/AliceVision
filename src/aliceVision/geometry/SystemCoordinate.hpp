// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ALICEVISION_GEOMETRY_SYSTEMCOORDINATE_HPP_
#define ALICEVISION_GEOMETRY_SYSTEMCOORDINATE_HPP_

#include <map>

namespace aliceVision
{
namespace geometry
{
namespace SystemCoordinate
{
Mat3 rot_arcore_to_alice()
{
    Mat3 transform_matrix;
    transform_matrix << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    return transform_matrix;
}

Mat3 rot_pytorch_to_alice()
{
    Mat3 transform_matrix;
    transform_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    return transform_matrix;
}

enum SYSTEM
{
    ALICE = 0,
    ARCORE = 1,
    PYTORCH = 2
};

const std::map<std::string, SystemCoordinate::SYSTEM> stringToSYSTEM = {
    {"alice", SYSTEM::ALICE}, {"arcore", SYSTEM::ARCORE}, {"pytorch", SYSTEM::PYTORCH}};

const std::map<SystemCoordinate::SYSTEM, std::string> SYSTEMToString = {
    {SYSTEM::ALICE, "alice"}, {SYSTEM::ARCORE, "arcore"}, {SYSTEM::PYTORCH, "pytorch"}};

Pose3 transform(const Pose3& pose, const SYSTEM& inputSystem, const SYSTEM& outputSystem)
{
    Pose3 visionPose = pose;
    switch(inputSystem)
    {
        case SYSTEM::ARCORE:
            visionPose.rotation() = pose.rotation() * rot_arcore_to_alice();
            break;
        case SYSTEM::PYTORCH:
            visionPose.rotation() = pose.rotation() * rot_pytorch_to_alice();
            break;
        default:
            break;
    }

    Pose3 outputPose = visionPose;
    switch(outputSystem)
    {
        case SYSTEM::ARCORE:
            outputPose.rotation() = visionPose.rotation() * rot_arcore_to_alice().transpose();
            break;
        case SYSTEM::PYTORCH:
            outputPose.rotation() = visionPose.rotation() * rot_pytorch_to_alice().transpose();
            break;
        default:
            break;
    }
    return outputPose;
}
Pose3 transform(const Pose3& pose, const std::string& inputStringSystem, const std::string& outputStringSystem)
{
    SYSTEM inputSystem = stringToSYSTEM.at(inputStringSystem);
    SYSTEM outputSystem = stringToSYSTEM.at(outputStringSystem);
    return transform(pose, inputSystem, outputSystem);
}
} // namespace SystemCoordinate

} // namespace geometry
} // namespace aliceVision

#endif // ALICEVISION_GEOMETRY_SYSTEMCOORDINATE_HPP_