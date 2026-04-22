# Build Crash Solutions for Raspberry Pi

## Problem Summary
Your Pi is freezing during compilation because:
- **Total Available RAM**: ~1.8GB
- **VSCode Server**: ~730MB (40% of available memory!)
- **GNOME Desktop**: ~300MB+ (17% of available memory)
- **Free for Building**: <800MB
- **Needed for C++ compilation**: 500MB-1GB per process

Even single-threaded builds exhaust memory and crash the system before Linux's OOM killer can react.

---

## RECOMMENDED SOLUTIONS (in order of effectiveness)

### Solution 1: Build via SSH Without Desktop (BEST)

1. **SSH into your Pi from another computer** (while leaving the GUI running for other work)
2. **Run the build in the SSH session:**
   ```bash
   cd ~/ROS2_Deploy/RTES
   ./build_minimal.sh
   ```

This works because SSH sessions don't load the GUI in their environment, freeing up memory for compilation.

### Solution 2: Build from Text Console (TTY)

1. **Switch to text console:** Press `Ctrl+Alt+F3`
2. **Login** with your credentials
3. **Run the build:**
   ```bash
   cd ~/ROS2_Deploy/RTES
   ./build_minimal.sh
   ```
4. **Switch back to GUI:** Press `Ctrl+Alt+F1` or `Ctrl+Alt+F7`

Text consoles don't load GNOME, saving 300MB+ RAM.

### Solution 3: Close VSCode and Build (MODERATE)

1. **Close VSCode completely** (File → Exit)
2. **Open a terminal** (Ctrl+Alt+T)
3. **Run:**
   ```bash
   cd ~/ROS2_Deploy/RTES
   ./build_minimal.sh
   ```

This frees 700MB+ but GNOME still consumes memory.

### Solution 4: Temporary Disable Desktop (ADVANCED)

```bash
sudo systemctl stop gdm3  # Stop desktop manager
cd ~/ROS2_Deploy/RTES
./build_minimal.sh
sudo systemctl start gdm3  # Restart desktop
```

Frees maximum memory but requires text-mode login.

---

## Available Build Scripts

### 1. `build_minimal.sh` (NEW - RECOMMENDED)
- Kills non-essential GUI apps
- Clears system caches
- Uses `-O0` (no optimization) to minimize compiler memory
- Single-threaded
- **Slowest but safest**

### 2. `build_and_test_safe.sh`
- Checks memory before building
- Single-threaded
- Debug mode
- Interactive test prompts

### 3. `build_and_test.sh` (UPDATED)
- Now uses safe single-threaded mode
- Was causing crashes before the fix

---

## Alternative: Build on Another Machine

If builds continue failing, consider:

1. **Cross-compilation** from a more powerful Linux machine
2. **Build in Docker** with memory limits
3. **Use a larger swap file** (not recommended - very slow)
4. **Upgrade RAM** if your Pi model supports it

---

## Quick Start (Try This First)

**If you're currently in VSCode (which you are):**

Open a **new terminal window outside VSCode**:
1. Press `Ctrl+Alt+T` to open GNOME Terminal
2. Run:
   ```bash
   cd ~/ROS2_Deploy/RTES
   ./build_minimal.sh
   ```

**OR SSH from another computer:**
```bash
ssh trashbot@<your-pi-ip>
cd ~/ROS2_Deploy/RTES
./build_minimal.sh
```

---

## What Was Changed

All build scripts now use:
- `--parallel-workers 1` - Prevents multiple compiler processes
- `-DCMAKE_BUILD_TYPE=Debug` - Uses less memory than Release
- `-O0` - No optimization (minimal.sh only)
- `-g0` - No debug symbols (minimal.sh only)
- Memory cleanup before building

---

## Monitoring Build Progress

In another terminal:
```bash
watch -n 2 free -h
```

This shows real-time memory usage. If "available" drops below 200MB, the system may freeze.
