# mc_kinova patch for the Gen3 6DOF

Upstream `isri-aist/mc_kinova` only ships the 7DOF arm. This folder has
everything needed to add the `Kinova6DOF` robot module used by
`KinovaHandGuiding` (`MainRobot: Kinova6DOF`).

- `mc_kinova_6dof.patch`: source changes to `src/kinova.cpp`, `src/kinova.h`
  and `src/module.cpp`:
  - registers the `Kinova6DOF` module and handles the missing `joint_7`
  - uses `bicep_link` in the self-collision list
  - **joint limits:** joint_1, joint_4 and joint_6 stay unbounded
    (infinite-rotation actuators, `continuous` in the URDF). joint_2 is
    ±2.15, and joint_3 (±2.57) and joint_5 (±2.09) come from the URDF.
    Before this change, the 7DOF overrides were applied, which clamped
    joint_4 to ±2.45 and joint_6 to ±2.0 rad.
- `share/`: 6DOF URDF, RSDF and convex hulls. The mc_kinova CMake does not
  generate them, so they must be copied into `<prefix>/share/mc_kinova/`.

The patch is made against upstream commit **7cf7424**. It does not apply to
newer upstream commits.

## Apply

```bash
cd /home/vscode/workspace/build/superbuild/src/mc_kinova   # or any mc_kinova checkout
git checkout 7cf7424
git apply /home/vscode/workspace/patches/mc_kinova/mc_kinova_6dof.patch
cmake --build /home/vscode/workspace/build/superbuild/build/mc_kinova
cmake --install /home/vscode/workspace/build/superbuild/build/mc_kinova
cp -r /home/vscode/workspace/patches/mc_kinova/share/* /usr/local/share/mc_kinova/
```

The bridge links against the mc_rtc in `/usr/local`, so it loads
`/usr/local/lib/mc_robots/kinova.so`, the superbuild install above.

## Regenerate the patch after editing mc_kinova

```bash
git -c safe.directory='*' -C /home/vscode/workspace/devel/mc_kinova diff 7cf7424 -- src \
  > /home/vscode/workspace/patches/mc_kinova/mc_kinova_6dof.patch
```
