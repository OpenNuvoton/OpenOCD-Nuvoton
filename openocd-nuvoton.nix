{ stdenv, lib, fetchurl, libftdi1, libusb, pkgconfig, autoreconfHook }:

stdenv.mkDerivation rec {
  name = "OpenOCD-Nuvoton";

  src = ./.;

  nativeBuildInputs = [ pkgconfig autoreconfHook ];
  buildInputs = [ libftdi1 libusb ];

  configureFlags = [
    "--enable-jtag_vpi"
    "--enable-usb_blaster_libftdi"
    (lib.enableFeature (! stdenv.isDarwin) "amtjtagaccel")
    (lib.enableFeature (! stdenv.isDarwin) "gw16012")
    "--enable-presto_libftdi"
    "--enable-openjtag_ftdi"
    (lib.enableFeature (! stdenv.isDarwin) "oocd_trace")
    "--enable-buspirate"
    (lib.enableFeature stdenv.isLinux "sysfsgpio")
    "--enable-remote-bitbang"
  ];

  NIX_CFLAGS_COMPILE = lib.optionals stdenv.cc.isGNU [
    "-Wno-implicit-fallthrough"
    "-Wno-format-truncation"
    "-Wno-format-overflow"
    "-Wno-unused-but-set-variable"
    "-Wno-strict-prototypes"
    "-Wno-implicit-int"
    "-Wno-sign-compare"
    "-Wno-maybe-uninitialized"
    "-Wno-unused-variable"
    "-Wno-incompatible-pointer-types"
    "-Wno-type-limits"
    "-Wno-tautological-compare"
  ];

  postInstall = lib.optionalString stdenv.isLinux ''
    mkdir -p "$out/etc/udev/rules.d"
    rules="$out/share/openocd/contrib/60-openocd-nuvoton.rules"
    if [ ! -f "$rules" ]; then
        echo "$rules is missing, must update the Nix file."
        exit 1
    fi
    ln -s "$rules" "$out/etc/udev/rules.d/"
  '';

  meta = with lib; {
    description = "Free and Open On-Chip Debugging, In-System Programming and Boundary-Scan Testing";
    longDescription = ''
      Nuvoton customized OpenOCD

      OpenOCD provides on-chip programming and debugging support with a layered
      architecture of JTAG interface and TAP support, debug target support
      (e.g. ARM, MIPS), and flash chip drivers (e.g. CFI, NAND, etc.).  Several
      network interfaces are available for interactiving with OpenOCD: HTTP,
      telnet, TCL, and GDB.  The GDB server enables OpenOCD to function as a
      "remote target" for source-level debugging of embedded systems using the
      GNU GDB program.
    '';
    homepage = https://github.com/OpenNuvoton/OpenOCD-Nuvoton;
    license = licenses.gpl2Plus;
    maintainers = with maintainers; [ rajivr ];
    platforms = platforms.unix;
  };
}
