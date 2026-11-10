/** @file
  GDI on the Mali-T628: the fixed GPU memory layout of the GDI render
  passes, shared by the host generator (ext/gpugen/gen_gdi.c, which packs
  the constant descriptors with Mesa's GenXML v4 packers into
  gdigpu_blob.h) and the driver (gdigpu_build.c patches the per-pass fields).

  Everything lives at fixed GPU addresses that exynoswddm reserves for
  itself at start: the scratch (shaders, descriptors, vertex data) inside
  one 16 MB executable window, the polygon list and the tiler heap above.

  One render pass = framebuffer = the bounding box of the draws'
  rectangles in the destination (left aligned down to 16 pixels so the
  colour writeback base stays 64-byte aligned):
    JS1: WRITE_VALUE (zero the polygon list header) -> TILER preload (a
         quad over the whole framebuffer copying the destination into the
         tile buffer: tiles are written back whole) -> one TILER job per
         draw (up to GDIGPU_MAX_DRAWS GDI operations on that destination,
         in GDI's order)
    JS0: FRAGMENT over the framebuffer's tiles.
  A draw lives in a slot; the generator packs every (slot, op) slot with
  Mesa's packers (gdigpu_blob.h mGdiGpuSlots), the driver copies the one
  it needs and patches the per-pass fields.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __GDIGPU_LAYOUT_H__
#define __GDIGPU_LAYOUT_H__

#define GDIGPU_BASE           0xEE000000ull    // scratch: 64 KB (inside the 16 MB window 0xEE000000..0xEEFFFFFF)
#define GDIGPU_SCRATCH_SIZE   0x10000
#define GDIGPU_TEMP_BASE      0xEE010000ull    // copy within one surface: surface -> temp -> surface (GPU only)
#define GDIGPU_TEMP_SIZE      0xFF0000         // rest of the 16 MB window: a 2560x1600 32 bpp box fits
#define GDIGPU_POLY_BASE      0xEF000000ull    // polygon list (header + body), up to GDIGPU_POLY_SIZE
#define GDIGPU_POLY_SIZE      0xC00000
#define GDIGPU_HEAP_BASE      (GDIGPU_POLY_BASE + GDIGPU_POLY_SIZE)
#define GDIGPU_HEAP_SIZE      0x400000         // ends at 0xF0000000 = EX_GPU_VA_END
#define GDIGPU_RESERVED_BASE  GDIGPU_BASE      // what exynoswddm keeps out of its allocator
#define GDIGPU_RESERVED_END   (GDIGPU_HEAP_BASE + GDIGPU_HEAP_SIZE)

// --- offsets in the scratch ------------------------------------------------------------------------------------
#define GO_SH_COPY        0x0000      // copy.frag (texture -> colour), first tag 5
#define GO_SH_FILL        0x0100      // fill.frag (flat colour varying)
#define GO_SH_ALPHA       0x0200      // alpha.frag (texture x flat constant-alpha varying)
#define GO_RSD_PRE        0x0400      // RENDERER_STATE of the preload (= copy)
#define GO_SAMPLER        0x0500      // nearest, unnormalized coordinates, clamp to edge
#define GO_VIEWPORT       0x0540
#define GO_FBD            0x0600      // FRAMEBUFFER (SFBD, 320 bytes)
#define GO_WV             0x0780      // WRITE_VALUE_JOB (44)
#define GO_TILER_PRE      0x0800      // TILER_JOB (192): preload
#define GO_FRAG           0x0B00      // FRAGMENT_JOB (48)
#define GO_TEXPTR_DST     0x0B40      // u64: -> GO_TEX_DST
#define GO_TEX_DST        0x0B80      // TEXTURE (32) + SURFACE_WITH_STRIDE (16): the destination (preload)
#define GO_VARY_PRE       0x0C00      // ATTRIBUTE: varying 0 = uv (preload)
#define GO_VBUF_PRE       0x0C40      // ATTRIBUTE_BUFFER (16): -> GO_UV_PRE
#define GO_SAMPLER3       0x0D00      // SAMPLER x 3 (= GO_SAMPLER): cleartype samples 3 textures
#define GO_POS_PRE        0x1000      // vec4 x 6: the preload quad
#define GO_UV_PRE         0x1080      // vec4 x 6
#define GO_SH_CT          0x1100      // cleartype.frag (up to GDIGPU_SH_CT_MAX bytes)
#define GDIGPU_SH_CT_MAX  0x0F00
#define GO_POS_OP         0x2000      // vec4 x 6 x GDIGPU_MAX_RECTS, shared by the draws (each from an even rectangle)
#define GO_UV_OP          0x4000
#define GO_COL_OP         0x6000
#define GO_CA_OP          0x8000      // vec4 (Ca, Ca, Ca, Ca) / 255 per vertex
#define GO_SLOT(i)        (0xA000 + (i) * SL_SIZE) // draw i of the pass (op tiler job and its descriptors)
#define GDIGPU_BLOB_SIZE  (0xA000 + GDIGPU_MAX_DRAWS * SL_SIZE)  // the template (the rest of the scratch is unused)

// inside a slot
#define SL_TILER          0x000       // TILER_JOB (192), index 3 + i, depends on the previous job
#define SL_RSD            0x0C0       // RENDERER_STATE (64)
#define SL_TEXPTR         0x100       // u64 x 3: -> SL_TEX (and for cleartype -> GO_TEX_DST, -> SL_TEX2)
#define SL_TEX            0x140       // TEXTURE + SURFACE_WITH_STRIDE: the source (cleartype: the coverage)
#define SL_TEX2           0x200       // cleartype: the gamma row (R8 x 512)
#define SL_VARY           0x180       // ATTRIBUTE x 3
#define SL_VBUF           0x1C0       // ATTRIBUTE_BUFFER x 3: an array, records 16 bytes apart (the GPU indexes it)
#define SL_SIZE           0x280

#define GDIGPU_MAX_RECTS  64          // 6 vertices x 16 bytes x 64 = 0x1800 per array
#define GDIGPU_MAX_DRAWS  8
#define GDIGPU_MAX_DIM    8192
#define GDIGPU_MASK       0xFF        // tiler hierarchy (Mesa's default on Midgard)

// Job chain in JS1: WV (index 1) -> preload (2) -> draw 0 (3) -> draw 1 (4) ...; JS0: fragment.
#define GDIGPU_OP_COPY    1
#define GDIGPU_OP_FILL    2
#define GDIGPU_OP_ALPHAP  3           // AlphaBlend, SourceHasAlpha (premultiplied), unstretched
#define GDIGPU_OP_ALPHAC  4           // AlphaBlend, constant alpha only, unstretched
#define GDIGPU_OP_CT      5           // ClearTypeBlend with a gamma table: reads the destination from memory,
                                      // so only the first draw of a pass
#define GDIGPU_OPS        5           // slot templates per slot: one per op

#endif // __GDIGPU_LAYOUT_H__
