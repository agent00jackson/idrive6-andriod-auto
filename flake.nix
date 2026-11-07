{
  description = "NBT-AAD built with the bmw-tools LLVM QNX SDK and CMake";
  inputs.bmw-tools.url = "github:agent00jackson/bmw-tools";
  inputs.nixpkgs.follows = "bmw-tools/nixpkgs";
  outputs = { self, nixpkgs, bmw-tools }: let
    eachSystem = nixpkgs.lib.genAttrs [ "aarch64-darwin" "x86_64-darwin" "aarch64-linux" "x86_64-linux" ];
  in {
    packages = eachSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};
      sdk = bmw-tools.packages.${system}.qnx-sdk;
    in rec {
      nbt-aad = pkgs.stdenvNoCC.mkDerivation {
        pname = "nbt-aad";
        version = "milestone-7-llvm";
        src = pkgs.lib.cleanSourceWith {
          src = ./.;
          filter = path: type: let
            name = baseNameOf path;
            rel = pkgs.lib.removePrefix (toString ./. + "/") (toString path);
          in !(builtins.elem name [ ".git" "build" "result" "Makefile" "common.mk" ".project" ".cproject" ])
             && !(builtins.elem rel [ "milestone-7/lib" "milestone-7/rhmiapp/lib" ])
             && !(type == "symlink" && pkgs.lib.hasPrefix "result" name)
             && !(pkgs.lib.hasSuffix ".o" name || pkgs.lib.hasSuffix ".d" name);
        };
        nativeBuildInputs = [ pkgs.cmake pkgs.ninja pkgs.llvmPackages.llvm ];
        cmakeFlags = [ "-DCMAKE_TOOLCHAIN_FILE=${sdk}/share/cmake/QnxSdk/toolchain.cmake" "-DCMAKE_BUILD_TYPE=Release" ];
        doCheck = true;
        checkPhase = ''
          runHook preCheck
          for program in milestone-7/nbt-aad milestone-7/test_display milestone-7/test_controls milestone-7/wfd_probe milestone-7/pf_aa milestone-7/plugin_check; do
            llvm-readelf -h -l "$program" > elf.txt
            grep -q 'EXEC (Executable file)' elf.txt
            grep -q '/usr/lib/ldqnx.so.2' elf.txt
          done
          for plugin in milestone-7/plugin/libhmiaad.so milestone-7/rhmiapp/libNbtAad.so; do
            llvm-readelf -h "$plugin" | grep -q 'DYN (Shared object file)'
          done
          runHook postCheck
        '';
        postInstall = ''
          cd "$out"
          find bin share -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
        '';
        dontFixup = true;
      };
      default = nbt-aad;
      qnx-sdk = sdk;
    });
    devShells = eachSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};
      sdk = bmw-tools.packages.${system}.qnx-sdk;
    in {
      default = pkgs.mkShellNoCC {
        packages = [ pkgs.cmake pkgs.ninja pkgs.llvmPackages.llvm ];
        QNX_TOOLCHAIN_FILE = "${sdk}/share/cmake/QnxSdk/toolchain.cmake";
      };
    });
    checks = eachSystem (system: { cmake-build = self.packages.${system}.nbt-aad; });
  };
}
