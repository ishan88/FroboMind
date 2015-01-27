## *********************************************************
## 
## File autogenerated for the velocity_control package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 231, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 260, 'description': 'Proportional gain for linear velocity controller', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'lin_p', 'edit_method': '', 'default': 0.4, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Integral gain for linear velocity controller', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'lin_i', 'edit_method': '', 'default': 0.6, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Differential gain for linear velocity controller', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'lin_d', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Proportional gain for angular velocity controller', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'ang_p', 'edit_method': '', 'default': 0.8, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Integral gain for angular velocity controller', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'ang_i', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Differential gain for angular velocity controller', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'ang_d', 'edit_method': '', 'default': 0.05, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Common anti-windup max for integrators', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'integrator_max', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

