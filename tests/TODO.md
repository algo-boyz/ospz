**Roadmap**

| Priority | Item | Notes |
|----------|------|--------|
| 1 | **Save path** | `save_spz` / `pack_gaussians` → ZSTD streams → NGSP header + TOC |
| 2 | **Round-trip test** | boat → save → load → compare `num_points`, SH degree, sample positions/alphas |
| 3 | Extensions | ILV zone only if you need it |
| 4 | Multi-thread zstd | Optional later via ozstd |

Save path: 

1. `pack_gaussians` (cloud → `Packed_Gaussians`) — check in `pack.odin`  
2. Serialize packed buffers in attribute order  
3. ZSTD-compress each stream  
4. Write header + TOC + concatenated streams  