# Copyright 2021 Open Rise Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
from clearpath_config.common.types.file import File
from clearpath_generator_common.description.generator import (
    DescriptionGenerator
)


class ClearpathConfigUpdater:

    def __init__(
            self,
            config_file: str,
            output_path: str = "/etc/clearpath",
            ) -> None:
        self.config_file = os.path.realpath(config_file)
        self.output_path = os.path.realpath(output_path)
        self.dirs = {os.path.dirname(self.config_file)}
        self.doc = None

    def get_robot_description(self):
        """Get Robot Description Parameter"""
        return self.doc.toprettyxml(indent="  ")

    def is_file(self, path: str) -> bool:
        """Check if File is the Same"""
        return os.path.realpath(path) == self.config_file

    def update(self) -> None:
        """Re-load File and Create Description"""
        # Generate URDF
        dg = DescriptionGenerator(
            setup_path=self.output_path
        )
        dg.generate()

        # Re-load Description
        self.doc = xacro.process_file(
            os.path.join(self.output_path, "robot.urdf.xacro")
        )
