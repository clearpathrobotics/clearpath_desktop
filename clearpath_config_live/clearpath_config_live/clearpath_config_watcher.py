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

from typing import Set

from clearpath_config_live.clearpath_config_updater import (
    ClearpathConfigUpdater
)
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer


class ClearpathConfigWatcher:

    def __init__(self, setup_path: str, logger) -> None:
        self.observer = Observer()
        self.updater = ClearpathConfigUpdater(setup_path, logger)
        self.logger = logger

    @property
    def watched(self) -> Set[str]:
        """Get Directories being Watched."""
        return {emitter.watch.path for emitter in self.observer.emitters}

    def start(self, event_handler: FileSystemEventHandler) -> None:
        """Start Tracking Clearpath Config."""
        self.observer.start()
        self.update(event_handler)

    def stop(self) -> None:
        """Stop Tracking Clearpath Config."""
        self.observer.stop()
        self.observer.join()

    def update_watched(self, event_handler: FileSystemEventHandler) -> None:
        """Update Directories being Watched."""
        self.observer.unschedule_all()
        for path in self.updater.paths:
            self.logger.info('Watching directory: %s' % dir)
            self.observer.schedule(event_handler, path=path, recursive=False)

    def update(self, event_handler: FileSystemEventHandler) -> None:
        """Update Clearpath Config and Watchlist."""
        self.updater.update()
        self.update_watched(event_handler)
