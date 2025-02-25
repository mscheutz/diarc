{
  description = "BoundMPCDiarc";

  nixConfig = {
    substituters = [
      "https://nix-community.cachix.org"
      "https://cache.nixos.org/"
      "https://ros.cachix.org"
    ];
    trusted-public-keys = [
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    ros-flake.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "ros-flake/nixpkgs";
    # arc.url = "path:/home/toeleric/research/arc";
    hsl.url = "path:/home/toeleric/source/coinhsl-2015.06.23";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      ros-flake,
      hsl,
    }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ ros-flake.overlays.default ];
        config.allowUnfree = true;
      };
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        shellHook = ''
          source "$(get_ros2_path).bash"
          export QT_QPA_PLATFORM=xcb
          export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${
            with pkgs;
            lib.makeLibraryPath [
              xorg.libX11
              xorg.libXt
              xorg.libSM
              zlib
              glib
              udev
              libGL
              glfw
              boost
              libusb1
              gmp
              portaudio
            ]
          }:${pkgs.stdenv.cc.cc.lib}/lib"
            source .venv/bin/activate
            export PYTHONPATH="$PYTHONPATH:$VIRTUAL_ENV/lib/python3.12/site-packages"
            export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$LIBNVIDIA_PATH"
            export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${hsl}/result/lib"
            export BOSDYN_CLIENT_USERNAME=hrilab
            export BOSDYN_CLIENT_PASSWORD=hrilabrulesspotphi
            export PYTHONPATH="/home/toeleric/research/boundmpc_speech/src/diarc/core/src/main/python:$PYTHONPATH"
            export TRADE_PROPERTIES_PATH="/home/toeleric/research/boundmpc_speech/src/diarc/core/src/main/python/pytrade/local_trade_hub.properties"
            export JAVA_HOME=$(readlink -e $(type -p javac) | sed  -e 's/\/bin\/javac//g')
            unset QT_PLUGIN_PATH
        '';
        name = "BoundMPC";
        buildInputs =
          with pkgs;
          with pkgs.rosPackages.humble;
          [
            (buildEnv {
              paths = [
                texliveFull
                boost
                # arc.packages.x86_64-linux.arc_lib
                # arc.packages.x86_64-linux.arc_core
                # arc.packages.x86_64-linux.arc_py
                ros-core
                ros2run
                ros2launch
                colcon
                geometry-msgs
                sensor-msgs
                rviz2
                xacro
                robot-state-publisher
                joint-state-publisher
                joint-state-publisher-gui
                ament-cmake-core
                python-cmake-module
                rosidl-default-generators
                ament-cmake
                std-msgs
                geometry-msgs
                hardware-interface
                controller-interface
                controller-manager
                ros2-control-test-assets
                joint-state-broadcaster
                joint-trajectory-controller
                gripper-controllers
                ros2-control

                # For pydrake
                xorg.libX11
                xorg.libXt
                xorg.libSM
                zlib
                glib

                ipopt

                portaudio

                eigen
                pyright
                ruff
                ruff-lsp

                python312Packages.pip
                python312Packages.tkinter
                python312Packages.cmake
                python312Packages.pinocchio
                python312Packages.hpp-fcl
                python312Packages.eigenpy
                python312Packages.isort
                python312Packages.debugpy
                cmake
                gcc
                stdenv
                udev
                libGL
                glfw
                graphviz
                gmp
                #glibc.static
              ];
            })
          ];
      };
    };
}
