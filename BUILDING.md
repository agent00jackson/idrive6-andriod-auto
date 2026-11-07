# Build with Nix and CMake

Use branch `codex/llvm-cmake-sdk`. The build imports the QNX SDK from
`bmw-tools` as a Nix input and builds the application with CMake/Ninja and
Clang/LLD. It does not use Momentics, QNX project files, a QNX license,
Docker, the retired Python builder, or the prebuilt libraries in this repo.

While the SDK changes are unpublished, keep the two checkouts next to each
other (`bmw-tools` and `id6-aa`), then run:

```sh
./build.sh --out-link result-llvm
```

The wrapper runs this Nix command, selecting the sibling SDK input:

```sh
nix build path:.#nbt-aad \
  --override-input bmw-tools path:/absolute/path/to/bmw-tools \
  --out-link result-llvm
```

Set `BMW_TOOLS_SOURCE` if the SDK checkout is elsewhere. After the SDK is
published, the flake's default input is `github:agent00jackson/bmw-tools`;
lock that published revision with `nix flake lock` before distributing a
release. Local development overrides intentionally do not write a lock
containing your machine's absolute checkout path. The SDK's nixpkgs input
is pinned by the bmw-tools lock file.

For an ordinary editable CMake build:

```sh
nix develop path:. --override-input bmw-tools path:/absolute/path/to/bmw-tools
cmake -S . -B build/llvm -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$QNX_TOOLCHAIN_FILE" -DCMAKE_BUILD_TYPE=Release
cmake --build build/llvm
cmake --install build/llvm --prefix "$PWD/build/release"
```

`cmake --build build/llvm --target nbt-aad NbtAad` builds just the daemon
and native iDrive app. CMake generates `build/llvm/compile_commands.json`.
The source list in `milestone-7/CMakeLists.txt` is authoritative; add new
translation units there.

The Nix output contains:

- `bin/nbt-aad` and the five diagnostic executables
- `share/nbt-aad/rhmiapp/libNbtAad.so` and its XML/manifest/text/image resources
- `share/nbt-aad/hmiplugins/libhmiaad.so` and the optional legacy plugin resources
- licenses under `share/licenses` and a `SHA256SUMS` manifest

Only the application products are installed. The generated BMW link-only
interfaces stay in the build tree. Never deploy SDK import DSOs; the programs
use the original QNX/BMW libraries already installed on the car.

Run the full build checks with:

```sh
nix flake check path:. --override-input bmw-tools path:/absolute/path/to/bmw-tools
```

The old QNX Makefile, common.mk and toolchain configuration were archived in
`../bmw-tools-private/qnx/archive/id6-aa`. Host regression tests remain in
`milestone-7/tests`; they are independent of the QNX target build.

See [the LLVM update instructions](LLVM_UPDATE.md) when you are ready to
update the car. Building does not connect to or modify a vehicle. The
application remains under its existing PolyForm Noncommercial license.
