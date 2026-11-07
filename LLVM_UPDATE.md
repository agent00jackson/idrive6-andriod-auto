# Updating an existing installation to the LLVM build

Build the release as described in [BUILDING.md](BUILDING.md). The migration
has been checked by building/linking and inspecting target imports. These
binaries have not yet been executed on your head unit. Perform the first
runtime qualification with the vehicle parked and retain the existing files.

The build does not update the car automatically. Before changing anything,
copy the new release to a staging directory on the head-unit processor that
hosts `/opt/aa`. SSH connections through the other processor need the usual
QNET `/net/hu-omap` path or `on -f` command from the existing update guide.

Transfer these files from `result-llvm` to staging (USB is also suitable):

| Build file | Installed location |
| --- | --- |
| `bin/nbt-aad` | `/opt/aa/nbt-aad` |
| `share/nbt-aad/rhmiapp/libNbtAad.so` | `/opt/hmi/ID5/data/ro/Plugins/NbtAad/libNbtAad.so` |
| `share/nbt-aad/hmiplugins/libhmiaad.so` | Update only if this optional legacy plugin is already installed |

Keep the current `nbt-aad.env`, autostart settings and plugin XML/databases.
This migration changes the compiler and build system, not those settings.
The release includes XML/resources for a fresh installation; do not overwrite
a customized installed variant merely to change the compiler.

First run the staged daemon with `--license`. This exercises the QNX loader,
startup and basic C/C++ runtime without starting an Android Auto session.
A failure here is a reason to keep the current installation and capture the
error. Inspect dynamic dependencies with `ldd` where available. Do not add
SDK import libraries to satisfy a missing runtime dependency.

Stop Android Auto using the menu's Force Stop action and confirm its process
has exited. Remount the filesystem writable only for the file swap. Use a
unique backup directory outside the HMI Plugins search tree (for example,
`/opt/aa/backups/llvm-<your-date>`), preserving each existing executable and
the entire native plugin directory. Do not overwrite a running executable
or a loaded shared library's inode: rename the old binary to the backup
location, then rename the staged replacement into its installed path.

For the native plugin, preserve a complete backup of the current directory
outside `Plugins`, retaining its current resources. Replace only its binary
using a staged file and rename. Mark the new daemon executable. Verify the
copied file hashes against the local release, then restore the filesystem's
previous read-only state and reload iDrive using your established procedure.

After reloading, check the native Android Auto menu and daemon/plugin logs,
then test phone connection, audio, microphone, controls and video. Capture
logs before rebooting, because `/dev/shmem` logs are volatile. A successful
`--license` is only the initial runtime smoke test, not a phone-session test.

If startup or menu loading fails, stop the new daemon, rename the new files
aside, and restore the backed-up binaries and native plugin directory. Reload
iDrive. Settings and factory libraries should not need any changes.

No vehicle update was performed as part of preparing this build.
