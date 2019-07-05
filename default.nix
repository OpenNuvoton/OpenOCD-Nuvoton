{ nixpkgs ? import <nixpkgs> {} }:

nixpkgs.callPackage ./openocd-nuvoton.nix {}
