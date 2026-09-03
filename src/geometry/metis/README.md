# Embedded METIS

This directory contains the private C++ implementation of METIS used by
`uipc_geometry` for mesh partitioning. It includes the required GKlib-derived
support code and is built as the internal `uipc_metis` static target; it is not
part of libuipc's public C++ API.

The implementation exposes the METIS 5.2.1 API and uses 32-bit `idx_t` plus
64-bit `real_t`, matching the former in-tree C implementation. The
`mesh_partition` caller fixes `METIS_OPTION_SEED` to zero for deterministic
partitioning.

See `LICENSE-METIS` and `LICENSE-GKlib` for the upstream copyright and Apache
2.0 license notices. The unused glibc-derived getopt/regex/qsort compatibility
code and optional third-party random generator from upstream GKlib are not
included; the C++ port uses a deterministic in-tree median-of-three
partition/insertion sort and the existing C-runtime RNG path. The replacement
sort deliberately preserves the former ordering of equal keys so mesh
partitions do not change.
