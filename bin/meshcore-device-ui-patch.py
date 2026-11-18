"""Apply patches/device-ui-meshcore.patch to the device-ui library PlatformIO fetched into .pio/libdeps.

The screen software is a library, not part of this repository; the MeshCore build changes a few things in it (node
list, chats). A marker file in the library's folder holds the hash of the patch that was applied.

While changing the library by hand (then writing the patch from it, tools/dui_patch.py one folder up), put an empty
file .meshcore-editing into the library's folder: the check is skipped as long as it is there.
"""
Import("env")  # noqa: F821

import hashlib
import os
import subprocess

project = env.subst("$PROJECT_DIR")  # noqa: F821
lib = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"), "meshtastic-device-ui")  # noqa: F821
patch = os.path.join(project, "patches", "device-ui-meshcore.patch")
marker = os.path.join(lib, ".meshcore-patch")


def git_apply(*options):
    where = os.path.relpath(lib, project).replace(os.sep, "/")
    return subprocess.run(["git", "apply", "--whitespace=nowarn", "--directory=" + where, *options, patch], cwd=project,
                          capture_output=True, text=True)


def mark(digest):
    with open(marker, "w") as f:
        f.write(digest + "\n")


if os.path.isdir(lib) and os.path.isfile(patch):
    with open(patch, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    have = ""
    if os.path.exists(marker):
        with open(marker) as f:
            have = f.read().strip()
    if os.path.exists(os.path.join(lib, ".meshcore-editing")):
        print("device-ui: being edited by hand (.meshcore-editing), patch not checked")
    elif have == digest:
        pass
    elif git_apply("--check", "--reverse").returncode == 0:
        mark(digest)  # the library already has exactly these changes
    elif not have and git_apply("--check").returncode == 0:
        done = git_apply()
        if done.returncode:
            print("device-ui patch failed:\n" + done.stderr)
            env.Exit(1)  # noqa: F821
        mark(digest)
        print("device-ui: applied patches/device-ui-meshcore.patch")
    else:
        print("device-ui in %s does not match patches/device-ui-meshcore.patch (another version of the library or of "
              "the patch). Delete that folder and build again." % lib)
        env.Exit(1)  # noqa: F821
