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

from rclpy.task import Future
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler

from clearpath_config_live.clearpath_config_watcher import (
    ClearpathConfigWatcher
)
from clearpath_config_live.robot_description_client import (
    RobotDescriptionClient
)


class ClearpathConfigHandler(FileSystemEventHandler):

    def __init__(
            self,
            watcher: ClearpathConfigWatcher,
            client: RobotDescriptionClient,
            logger
            ) -> None:
        self.watcher = watcher
        self.client = client
        self.future = Future()
        self.logger = logger

    def on_modified(self, event):
        """Check for Updates to Relevant Files in Directory."""
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            if self.watcher.updater.is_file(event.src_path):
                try:
                    self.watcher.update(self)
                    self.future = self.client.call_async(
                        self.watcher.updater.get_robot_description()
                    )
                except Exception as ex:
                    self.future = Future()
                    self.logger.error("Updater failed to regenerate config:")
                    self.logger.error(str(ex))
