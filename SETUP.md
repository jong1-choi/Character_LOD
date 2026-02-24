# Character LOD - Setup Guide

Character animation Level of Detail (LOD) viewer using BVH motion capture data.
Renders the original skeleton, a simplified skeleton, and a keyframe-reduced skeleton side by side.

---

## Requirements

| Tool | Version | Notes |
|------|---------|-------|
| Visual Studio | 2019 or later | Workload: **Desktop development with C++** required |
| gnuplot | Any recent | Required by sciplot for keyframe graph visualization |

All other dependencies (GLEW, GLFW, GLM, Eigen, sciplot) are included in the `include/` and `lib/` folders.

### Install gnuplot (Windows)
1. Download from https://sourceforge.net/projects/gnuplot/
2. During installation, check **"Add application directory to PATH"**
3. Verify: open a new Command Prompt and run `gnuplot --version`

---

## Build

1. Open `Character_LOD.sln` in Visual Studio
2. Select configuration: **Debug x64** or **Release x64**
3. Build → Build Solution (`Ctrl+Shift+B`)

---

## BVH Data Path

The BVH motion capture data path is set in [Character_LOD/main.cpp](Character_LOD/main.cpp) line 11:

```cpp
const std::string bvhDir = "../Character_LOD/data/BVH_dance";
```

This is a relative path from the working directory (`Res/`).
If you move the project or use different BVH files, update this line accordingly.

---

## Run

### From Visual Studio (F5 / Ctrl+F5)
The working directory is pre-configured to `Res/` in `Character_LOD.vcxproj.user`.
Shaders load automatically — just press F5.

### From the built executable directly
Copy the four shader files into the same folder as the `.exe`:
```
Res/shader.vert
Res/shader.frag
Res/const.vert
Res/const.frag
```
Then run the `.exe` from that folder (or set its working directory to that folder).

---

## Controls

| Key | Action |
|-----|--------|
| `Space` | Play / Pause animation |
| `0` | Reset to frame 0 |
| `[` | Previous motion clip |
| `]` | Next motion clip |
| Left drag | Rotate camera |
| Scroll | Zoom in / out |

---

## Project Structure

```
Character_LOD-main/
├── Character_LOD.sln          Visual Studio solution
├── Res/                       Shader files (working directory at runtime)
│   ├── shader.vert / .frag    Main rendering shaders
│   └── const.vert / .frag     Constant color shaders
├── include/                   Third-party headers (Eigen, GL, GLFW, glm, sciplot)
├── lib/                       Pre-compiled static libraries (glew32s, glfw3)
└── Character_LOD/
    ├── data/BVH_dance/        BVH motion capture files
    ├── main.cpp               Entry point, BVH path config
    ├── motion.h / .cpp        Body/Link/Motion data structures
    ├── readBVH.h / .cpp       BVH file parser
    ├── MotionFitting.h / .cpp LOD skeleton fitting
    ├── KeyReduction.h / .cpp  Keyframe reduction algorithm
    ├── ModelView.h / .cpp     3D viewport widget
    ├── AnimView.h / .cpp      Animation playback widget
    ├── GLTools.h / .cpp       OpenGL utilities
    └── Widget.h               Base widget / input state
```

---

## Troubleshooting

**Shaders not found (`[ERROR] Text file: shader.vert is not found`)**
- Running from VS: confirm `Character_LOD.vcxproj.user` exists and has not been deleted
- Running the `.exe` directly: ensure the four shader files are in the same folder as the executable, and run from that folder

**BVH files not loading**
- Check `bvhDir` in `main.cpp` points to the correct folder
- The path is relative to the `Res/` working directory, so `../Character_LOD/data/BVH_dance` resolves to the `data/BVH_dance` folder inside the source project

**Keyframe graph window does not appear**
- gnuplot is not installed or not in PATH
- Install gnuplot and ensure it is accessible from the command line (`gnuplot --version`)
