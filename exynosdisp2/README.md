# exynosdisp2

Display-only (KMDOD, WDDM 1.2) driver for the v1awifi internal panel
(LSL122DL01 eDP on FIMD1 + DP1, LP8556 backlight); replaced the single-buffer
exynosdisp 1.0.0.x.  Built by ../build.sh as 2.0.0.N.

- Cold panel bring-up through the shared UEFI cores (cores.c); monitor D3 /
  D0 switch the panel completely off / on; brightness interface.
- Two own scanout buffers flipped through FIMD1's shadowed window 0 address
  at vsync (FIMD1 frame interrupt, combiner group 18 input 5 = GSI 50, from
  the DSDT); flips are armed under the interrupt lock.
- Window 1: 64x64 ARGB hardware cursor.
- Rotation: the copy follows the image's shape (portrait works; 180 degrees
  does not render correctly yet).
- Stop / release: the front buffer is copied into the firmware framebuffer
  and scanned out from there, the panel stays on.
