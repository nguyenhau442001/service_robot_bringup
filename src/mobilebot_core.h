/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef MOBILEBOT_CORE_H_
#define MOBILEBOT_CORE_H_

#define NAME                             "Service Robot"

#define WHEEL_RADIUS                     0.072             // meter
#define WHEEL_SEPARATION                 0.453             // meter 
#define TURNING_RADIUS                   0.2265            // meter 
#define ROBOT_RADIUS                     0.275             // meter 
#define ENCODER_MIN                      0                 // raw
#define ENCODER_MAX                      50000             // raw
#define RPM_MAX                          300               // 3000/10

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * RPM_MAX / 60) // m/s  
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //MOBILEBOT_CORE_H_