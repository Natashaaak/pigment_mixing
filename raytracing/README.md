# Scenes

Scenes are adjustable, you can control the amount of particles in the scene by changing "particleRadius" in configuration.
Scenes are in data/Scenes.

# Controls

You can control the simulation.
Use SPACE key to pause/play the simulation.
Use ESC key to exit.
Used WS keys to move closer/further from the fluid surfaces.

# Build and Installation Instructions

The project requires the following tools:
* **CMake >= 3.20**
* A C++ compiler with **C++20** support (MSVC, GCC, or Clang).
* **Git**.
* **OpenGL**.
* **OpenMP** support.

---

## Dependencies
The project uses the following libraries:
* **glad**
* **glm**
* **ImGui**
* **Eigen3**
* **spdlog** built from `external/spdlog`
* **GLFW**

---

## Build with vcpkg (recommended)
If vcpkg is not available, clone and bootstrap it:

**Linux:**
```bash
git clone [https://github.com/microsoft/vcpkg](https://github.com/microsoft/vcpkg)
./vcpkg/bootstrap-vcpkg.sh
```

**Windows:**
```powershell
git clone [https://github.com/microsoft/vcpkg](https://github.com/microsoft/vcpkg)
.\vcpkg\bootstrap-vcpkg.bat
```

From the project root (where `vcpkg.json` is located):
```bash
vcpkg install
```

Configure with the vcpkg toolchain:
```bash
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=<VCPKG_ROOT>/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --config Release --target GLHydroSurface
```

---

## Alternative: System Packages (Linux)
If vcpkg is not used, the system must have installed:
* **OpenGL**
* **OpenMP runtime**
* **glad**
* **glm**

Then configure and build normally:
```bash
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```