#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

CONFIG_FILE = "submodule_cfg.yaml"


def get_pip_command():
    """Detect if uv is available and return the appropriate pip command."""
    try:
        # Check if uv is available
        result = subprocess.run(
            ["uv", "--version"],
            capture_output=True,
            text=True,
            check=False
        )
        if result.returncode == 0:
            return "uv pip"
    except FileNotFoundError:
        pass
    return "pip"


def run(cmd, cwd=None):
    print(f"Running: {cmd}")
    try:
        subprocess.run(cmd, shell=True, cwd=cwd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command failed with error: {e}")


def load_config():
    with open(CONFIG_FILE) as f:
        return yaml.safe_load(f)


def install_submodules(selected=None):
    config = load_config()
    for name, info in config.items():
        print(f"\n----- Installing submodule: {name} -----")
        install = info.get("install", False)
        if (selected is None and not install) or (selected is not None and name not in selected):
            print(f"Skipping submodule: {name}")
            continue

        path = Path(info["path"])
        patches = info.get("patches", [])
        addons = info.get("addons", [])

        print(f"Initializing submodule '{name}'...")
        run(f"git submodule update --init {path}")

        if not path.exists():
            print(f"Path {path} does not exist. Skipping {name}.")
            continue

        for patch in patches:
            patch_path = path / patch
            if patch_path.exists():
                print(f"Applying patch {patch} for '{name}'...")
                run(f"cd {path} && git apply {patch}")
            else:
                print(f"Patch {patch} not found for '{name}', skipping.")

        for addon in addons:
            addon_path = path / addon
            if addon_path.exists():
                print(f"Adding addon '{addon_path}' to '{name}'...")
                shutil.copytree(addon_path, path, dirs_exist_ok=True)
            else:
                print(f"Addon path {addon} does not exist, skipping.")

        # Handle CMake build/install
        cmake_config = info.get("cmake", None)
        if cmake_config is not None:
            if cmake_config.get("build", False):
                build_dir = cmake_config.get("build_dir", "build")
                build_path = path / build_dir
                print(f"Building CMake project for '{name}' in {build_path}...")
                
                # Create build directory if it doesn't exist
                build_path.mkdir(exist_ok=True)
                
                # Run CMake configuration
                run(f"cmake ..", cwd=build_path)
                
                # Build
                num_cores = os.cpu_count() or 1
                run(f"make -j{num_cores}", cwd=build_path)
                print(f"CMake build complete for '{name}'")
            
            if cmake_config.get("install", False):
                build_dir = cmake_config.get("build_dir", "build")
                install_prefix = cmake_config.get("install_prefix", "/usr/local")
                build_path = path / build_dir
                
                # Ensure build directory exists and is configured
                if not build_path.exists():
                    print(f"Build directory {build_path} does not exist. Building first...")
                    build_path.mkdir(exist_ok=True)
                    run(f"cmake ..", cwd=build_path)
                    num_cores = os.cpu_count() or 1
                    run(f"make -j{num_cores}", cwd=build_path)
                
                print(f"Installing CMake project for '{name}' to {install_prefix}...")
                run(f"cmake -DCMAKE_INSTALL_PREFIX={install_prefix} ..", cwd=build_path)
                run(f"sudo make install", cwd=build_path)
                
                # Install Vicon SDK libraries to system library path if they exist
                libs_path = path / "libs"
                if libs_path.exists():
                    print(f"Installing Vicon SDK libraries to system library path...")
                    run(f"sudo cp {libs_path}/*.so /usr/lib/ 2>/dev/null || true")
                    run(f"sudo ldconfig")
                
                print(f"CMake installation complete for '{name}'")

        # install Python package
        packages = [path]  # main package
        if (extra_packages := info.get("extra_packages", None)) is not None:
            packages += extra_packages
        pip_cmd = get_pip_command()
        for pkg in packages:
            # Skip pip install if this is a CMake-only package
            if cmake_config is None or info.get("python_package", True):
                run(f"{pip_cmd} install -e {pkg}")


if __name__ == "__main__":
    selected_modules = sys.argv[1:] if len(sys.argv) > 1 else None
    # selected_modules will override install cfg if provided
    install_submodules(selected_modules)

    # Usage example:
    # python submodule_install.py mujoco_viewer unitree_cpp