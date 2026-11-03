# SimpleI2CTouchpad

Driver for an I2C HID precision touchpad on Intel LPSS (written for a GXT7863 I2C precision touchpad on Tiger Lake I2C1, address 0x2C).
It replaces VoodooI2C, VoodooI2CHID, VoodooGPIO and VoodooI2CServices. VoodooInput stays: it is the part that
makes macOS see a Magic Trackpad, so all macOS trackpad settings and gestures work.

How it works:
1. It drives the I2C controller itself.
2. It watches the touchpad's interrupt line by reading its GPIO pad (`GPP_E8`, `0x090B0008` from the DSDT) every 2 ms,
   instead of blindly polling. A report is read only when the touchpad has one.
3. It parses the touchpad's own HID report descriptor and switches it to precision-touchpad mode.
4. Every finger gets a proper "down" and "up". A finger that vanishes without an "up" is lifted, and if the touchpad
   goes silent for 300 ms all fingers are lifted. This prevents cursor jumps and lost clicks.

## Build
Push this folder to a GitHub repository, open Actions > "SimpleI2CTouchpad kext build" > Run workflow,
and download the `SimpleI2CTouchpad` artifact. It contains `SimpleI2CTouchpad.kext`.

## Install
1. Mount the EFI partition:
   `sudo diskutil mount disk0s1`, then `E=$(diskutil info disk0s1 | awk -F': *' '/Mount Point/{print $2}')`
2. Back up the current config: `cp "$E/EFI/OC/config.plist" "$E/EFI/OC/config-before-trackpad.plist"`
3. Copy the kext: `cp -R SimpleI2CTouchpad.kext "$E/EFI/OC/Kexts/"`
4. Edit your config.plist (Kernel > Add):
   - disable VoodooI2C.kext, VoodooI2CHID.kext, VoodooGPIO.kext and VoodooI2CServices.kext
   - keep VoodooInput.kext enabled (e.g. `VoodooI2C.kext/Contents/PlugIns/VoodooInput.kext`)
   - add `SimpleI2CTouchpad.kext` (ExecutablePath `Contents/MacOS/SimpleI2CTouchpad`) after VoodooInput
   - remove `-vi2c-force-polling` from boot-args
5. Restart.

## Go back
`cp "$E/EFI/OC/config-before-trackpad.plist" "$E/EFI/OC/config.plist"` and restart.
Boot-arg `-sitpoff` also stops the driver from attaching.

## Logs and status
- `log show --last boot --predicate 'eventMessage CONTAINS "SimpleI2CTouchpad"' > ~/Desktop/tp.log`
- `ioreg -rlw0 -c SimpleI2CTouchpad > ~/Desktop/tp-ioreg.txt` (contains the report descriptor and live counters)

## Settings (Info.plist of the kext, personality SimpleI2CTouchpad)
- `GPIOPad`: the touchpad's interrupt pad (Intel `0xGGGG_PPPP` form from the DSDT, here 0x090B0008).
- `DebugLevel`: 0 quiet, 1 frames, 2 frames and raw reports (first 150). Boot-arg `sitpdebug=N` overrides.
- `WatchdogMs`: lift all fingers after this much silence (0 = off).
- `ForcePolling`: 1 = ignore the GPIO line and poll (also boot-arg `-sitppoll`).

On another laptop, set `GPIOPad` to the pad your DSDT's touchpad `_INI`/`_CRS` uses (or `ForcePolling` = 1).
