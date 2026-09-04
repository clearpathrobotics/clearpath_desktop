# Contributing to clearpath_desktop

Thanks for your interest in improving `clearpath_desktop`! These packages run on a **ROS 2 desktop
/ offboard computer** — the operator's workstation rather than the robot itself — to visualize,
teleoperate, and consume data from a running Clearpath robot. Please read the notes below before
opening a pull request.

## Getting started

1. Fork the repository and clone your fork.
2. Create a feature branch off `jazzy`:

   ```bash
   git checkout -b my-feature jazzy
   ```

3. Build the workspace and source it:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Install the pre-commit hooks (one-time setup):

   ```bash
   pip install pre-commit
   pre-commit install
   ```

## Linting

This repository uses [pre-commit](https://pre-commit.com/) to run linting and formatting checks
(trailing whitespace, end-of-file, YAML/JSON checks, `markdownlint`, and `flake8`) before each
commit. Run them against the whole tree before pushing:

```bash
pre-commit run --all-files
```

## Where things live

See the [Packages section of the README](README.md#packages) for the full map. In short:

- [`clearpath_viz`](clearpath_viz) — RViz visualization launchers.
- [`clearpath_config_live`](clearpath_config_live) — live URDF updater that tracks the robot's
  current config.
- [`clearpath_offboard_sensors`](clearpath_offboard_sensors) — launch files for decompressing and
  consuming high-bandwidth sensor data offboard.

Keep changes in the package that matches the feature you are working on, and avoid unrelated
refactors in the same pull request.

## Testing your changes

Because these packages run **off** the robot, testing them requires a running robot or simulation
on the same network. A quick way to validate changes is to launch `clearpath_simulator` and confirm
the desktop tools discover and display its topics. If the desktop sees no topics, verify the ROS
domain / middleware settings match the robot's `system` settings in `robot.yaml`.

Build and run any package tests through `colcon`:

```bash
colcon build --symlink-install
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Continuous integration

[`clearpath_desktop_ci`](.github/workflows/ci.yml) runs on every pull request:

- **jazzy** (`build_and_test`) — builds and tests against the released `testing`/`main` repos.
- **Jazzy Clearpath Source** (`source_build`) — source build of `clearpath_config_live`,
  `clearpath_desktop`, and `clearpath_viz`.

Both jobs build against **released** dependencies and do not pull in upstream source branches, so
they are not affected by in-progress branches in other Clearpath repositories — they should pass on
their own. If a job fails, the cause is in this repository (or an already-released upstream
dependency), not an unmerged upstream branch.

## Submitting a pull request

1. Make sure the workspace builds and any tests pass.
2. Push your branch and open a pull request against `jazzy`.
3. Write a clear description of what the change does and why. Link any related issues.
4. Note how you verified the change (e.g. against the simulator or a specific platform).

## Reporting issues

Please open issues on the
[GitHub issue tracker](https://github.com/clearpathrobotics/clearpath_desktop/issues) and fill out
the bug report template, which walks you through the details we need to reproduce the problem.

## License

By contributing, you agree that your contributions will be licensed under the
[BSD-3-Clause license](LICENSE) that covers this project.
