# Independent backend preparation

Recorded experiments used Windows, Python 3.10 and UE 5.2. Use the commit in
`native/upstream.lock.json`; newer engine/upstream versions are not automatically
interchangeable. Preserve your working projects and compiled binaries.
Obtain upstream separately, retaining its licenses, then assemble NEW source trees:

```powershell
python tools/materialize_backend.py --upstream D:/research/ProjectAirSim --profile main --check-only
python tools/materialize_backend.py --upstream D:/research/ProjectAirSim --profile main --destination D:/research/backend-main
python tools/materialize_backend.py --upstream D:/research/ProjectAirSim --profile moving-deck --destination D:/research/backend-landing
```

Assembly reads pinned Git objects and overlays 9 main or 24 landing files. It
ignores dirty checkout files and refuses existing destinations. It does not
compile/download/install anything. Each profile is relative to the SAME base;
do not apply them sequentially. `changes.patch` offers a line-level alternative.
For deterministic patch bytes, use `git -c core.autocrlf=false apply changes.patch`.
On Windows, one upstream documentation image has a case-colliding name; assembly
retains one and records the omission in AIRSIMSGA_SOURCE_ASSEMBLY.json. Runtime
source filename collisions are rejected rather than silently skipped.

## Main profile

Follow the assembled upstream `build.cmd` / `build_windows.mk` in a VS x64 Native
Tools prompt. Relevant targets include `simlibs_debug`, `package_simlibs` and
`package_plugin`; acquire dependencies/model weights through upstream setup.
Do not clean/rebuild your original project. The release excludes the original
machine's root-CMake cache/mirror settings; record any build-only URL corrections.
Install the assembled plugin/new matching libraries into a COPY of your scene,
set that project's UE association and compile its plugin using a matching engine.
A clean-machine rebuild was not performed during packaging.

## Landing profile

A portable version of the independent component build is supplied:

```powershell
cmake -S native/build_support -B D:/research/build-landing -G "Visual Studio 17 2022" -A x64 -DAIRSIMSGA_BACKEND=D:/research/backend-landing -DAIRSIMSGA_SIMLIBS=D:/research/landing-scene/Plugins/ProjectAirSim/SimLibs -DAIRSIMSGA_DEPS=D:/research/deps
cmake --build D:/research/build-landing --config Debug --target simserver support_contact_tests firmware_disarm_tests fastphysics_regression
ctest --test-dir D:/research/build-landing -C Debug --output-on-failure
```

SimLibs supplies Eigen/json/NNG/ONNX Runtime; deps supplies msgpack-src,
tinygltf-src and googletest-src. These external dependencies are not included.
Match Debug/Release and CRT configuration to UE. Install the resulting libraries
using the upstream layout. The landing overlay's Unreal plugin files MUST also
be installed/compiled in the copied project; libraries alone lose collision and
camera fixes. This component recipe follows Windows/MSVC, not a validated Linux build.

Run support_contact_tests and firmware_disarm_tests with
`--gtest_output=json:<report.json>` and retain the actual build log. Start the
independent landing scene (9089/9090) and create local receipts:

```powershell
python tools/create_landing_receipts.py --backend D:/research/backend-landing --plugin D:/research/landing-scene/Plugins/ProjectAirSim/Binaries/Win64/UnrealEditor-ProjectAirSim.dll --build-log D:/research/build-landing/build.log --support-tests D:/research/build-landing/support.json --firmware-tests D:/research/build-landing/firmware.json --pid 12345 --output D:/research/landing-local-receipts
```

Use real PID/files. The tool checks sources, passing report JSON and the loaded
DLL. Hashes alone do not prove the build produced that binary. Set
`landing_build_receipt` to the generated build_receipt.json; runtime_receipt.json
must be beside it. After simulator restart, record its new PID/create time in a
NEW receipt folder. Do not transplant historical receipts or disable validation.

## Final recorded planar-dynamics landing

Use `--profile recorded-landing` for the final vessel dynamics and native changes.
Build it in a new directory using the same dependency and UE-plugin procedures.
The exact component recipe and vessel dynamics test source are also retained in
`recorded/landing/build_support/` and `recorded/landing/tests/`; adjust its local
dependency paths. See `recorded_release.md` for final controller and ports.
