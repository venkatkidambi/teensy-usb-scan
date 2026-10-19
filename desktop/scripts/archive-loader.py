"""Bundle the exact loader sources used for compilation, including our patch."""
import pathlib
import subprocess
import sys
import zipfile

source, destination, instructions = map(pathlib.Path, sys.argv[1:])
paths = subprocess.check_output(["git", "ls-files", "-z"], cwd=source).decode().split("\0")
revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source).decode().strip()
with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as archive:
    for name in filter(None, paths):
        # Read working-tree contents so the modern Windows HID include fix is
        # included, rather than archiving the unpatched upstream commit.
        archive.write(source / name, name)
    archive.write(instructions, "BUILDING.md")
    archive.writestr("SOURCE_REVISION.txt", f"Upstream: {revision}\nWindows HID includes adjusted for modern MinGW headers.\n")
