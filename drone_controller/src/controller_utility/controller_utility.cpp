/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "controller_utility.h"
namespace drone_control{

//Utility functions
ControllerUtility::ControllerUtility(void): switchValue(false), prevInput(false) { }
ControllerUtility::~ControllerUtility() {}

double ControllerUtility::map(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double ControllerUtility::limit( double in, double min, double max)
{
    if(in < min)
    {
      in = min;
    }
    if( in > max)
    {
      in = max;
    }
    return in;
}


bool ControllerUtility::GetSwitchValue(void)
{
        return switchValue;
}

bool ControllerUtility::UpdateSwitchValue(bool currInput)
{
        if (currInput != prevInput)
        {
            if (currInput)
            {
                switchValue = !switchValue;
            }

            prevInput = currInput;
        }

        return switchValue;
}


Eigen::Vector3d ControllerUtility::rotateGFtoBF(double GF_x, double GF_y, double GF_z, double GF_roll, double GF_pitch, double GF_yaw)
{ 

    Eigen::Matrix3d R_roll;
    Eigen::Matrix3d R_pitch;
    Eigen::Matrix3d R_yaw;
    Eigen::Matrix3d Rot;

    Eigen::Vector3d GF_(GF_x, GF_y, GF_z);
    Eigen::Vector3d BF_;

    R_roll << 1, 0, 0, 0, cos(GF_roll), -sin(GF_roll), 0, sin(GF_roll), cos(GF_roll);
    R_pitch << cos(GF_pitch), 0 , sin(GF_pitch), 0, 1, 0, -sin(GF_pitch), 0, cos(GF_pitch);
    R_yaw << cos(GF_yaw), -sin(GF_yaw), 0, sin(GF_yaw), cos(GF_yaw), 0, 0, 0, 1;

    Rot = R_yaw * R_pitch * R_roll;

    BF_ = GF_.transpose()*Rot;

    return BF_.transpose();

} 


}