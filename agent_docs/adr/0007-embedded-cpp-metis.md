# ADR 0007 - Embed the C++ METIS Implementation in Geometry

- Status: Accepted
- Date: 2026-09-03
- Owners: Geometry and build systems
- Implements: current `refactor-main` migration
- Supersedes: separate `external/METIS` and `external/GKlib` targets

## Context

Mesh partitioning only consumes METIS privately through
`src/geometry/mesh_partition.cpp`. The former build maintained complete C
source trees and two external targets for METIS and GKlib. A C++ port now lives
under `src/geometry/metis/`, including the METIS 5.2.1 API and the required
GKlib-derived support subset.

The initial port also carried unused legacy compatibility code: glibc-derived
getopt/regex implementations and qsort macros, plus an optional Mersenne
Twister implementation. Those components introduced additional LGPL and BSD
license obligations despite not being required by the configured partitioner.

## Decision

- Build `src/geometry/metis/*.cpp` as the private static `uipc_metis` target in
  both CMake and XMake, with PIC on Unix-like toolchains.
- Link `uipc_geometry` privately to `uipc_metis`; do not compile the same source
  files directly into `uipc_geometry`.
- Remove the root `external/` source tree and the old `GKlib`/`metis` targets.
- Keep the embedded API private. Public users continue to call
  `uipc::geometry::mesh_partition` rather than including `metis.h`.
- Use modern platform headers for MSVC, make diagnostic string parameters
  const-correct, use a clean deterministic C++ median-of-three
  partition/insertion implementation for the required sort wrappers, and retain
  the existing C-runtime random path. The replacement preserves the former
  ordering of equal keys; do not replace it with `std::sort` or restore the
  unused getopt/regex/qsort/RNG compatibility implementations.
- Preserve the METIS and GKlib Apache-2.0 copyright notices next to the embedded
  sources and in the project `NOTICE` file.
- Keep the retained debug-timing and out-of-core paths functional with portable
  C++ clocks and direct single-file removal; reject zero partition sizes,
  32-bit graph-capacity overflow, and invalid METIS result IDs before unsafe
  arithmetic or indexing.

## Consequences

METIS remains independently compilable and incrementally cached, but no longer
creates a top-level third-party dependency tree. CMake and XMake own the same
source set and target boundary. The source distribution carries only the
Apache-2.0 METIS/GKlib notices instead of unused non-Apache implementation
fragments.

The implementation still derives from METIS/GKlib and therefore must retain
their copyright and Apache license notices. Rewriting support pieces in C++
does not remove that obligation from the remaining derived core.

## Alternatives considered

- Continue building `external/METIS` and `external/GKlib`: rejected because it
  duplicates the newly maintained C++ implementation and dependency targets.
- Compile the C++ sources directly into `uipc_geometry`: rejected because it
  loses the domain-level target and caused duplicate XMake compilation when
  combined with `uipc_metis`.
- Keep the bundled glibc getopt/qsort/regex and optional MT19937-64 code:
  rejected because a compact in-tree C++ sort and the configured C-runtime RNG
  supply the required behavior without the extra licensing surface.

## Validation

- CMake and XMake independently build `uipc_metis` and link `uipc_geometry`.
- The public geometry suite passes 38 cases / 270 assertions, including
  deterministic `mesh_partition` and invalid-input regressions.
- Before deleting the external sources, three synthetic graph families matched
  the former libraries exactly in return code, edge cut, and every partition
  ID.
- The `fluffy_ball.msh` check exercises the complete public retry policy, not
  just one METIS call: 173,263 vertices and 782,647 unique edges are repeatedly
  partitioned until every partition satisfies the requested maximum size.
- On Windows/MSVC, old and new builds were byte-identical. Maximum size 256
  converged at block size 249 with 696 parts, edge cut 55,869, and SHA-256
  `637F4CE1A5EE0F13C642123F824701C5A84B7A0CC9E64C36D4CACEDCB173F264`;
  maximum size 16 converged at block size 13 with 13,328 parts, edge cut
  286,074, and SHA-256
  `7CB2F21626CCBD2EE8C248DE28A46DB484BDD7A99D821CF575F91A035EA18688`.
- On Linux/GCC, the same complete old/new comparison was byte-identical.
  Maximum size 256 converged at block size 249 with 696 parts, edge cut 55,140,
  and SHA-256
  `F7850F01A78A37E547A602429475767238BB1C4C1BD7FFF33D16C2FA0251CC23`;
  maximum size 16 converged at block size 13 with 13,328 parts, edge cut
  286,034, and SHA-256
  `191B2B8A38EDC0ADFBC30BA8DCD46B90E5410790877055767A9ED1B24CC1861D`.
  Cross-platform IDs are not expected to match because the retained default
  RNG path uses each platform's C runtime; parity is exact within each platform.
- Extended comparisons covered maximum sizes 8 and 4 on both
  `fluffy_ball.msh` and the larger animal-wall mesh `animal_well.msh`. The
  comparison decoded the result arrays in original vertex order and counted
  differing partition IDs; it did not infer equivalence from aggregate
  partition counts. Every row below has zero differing vertices. The SHA-256
  covers the return code, edge cut, part count, and the full per-vertex ID
  array.

  | Platform | Mesh | Max size | Final block | Parts | Edge cut | Vertex mismatches | SHA-256 |
  | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
  | Windows/MSVC | `fluffy_ball.msh` | 8 | 6 | 28,878 | 455,924 | 0 | `281E3F71068345FCD7C8A35CF372ED457A31C23DD343B4C38487B5A104AA3D8B` |
  | Windows/MSVC | `fluffy_ball.msh` | 4 | 3 | 57,755 | 623,436 | 0 | `A9CD70D6BFC91F9145A79B71862DF7929C75123FE162BFFFC552AAA4B7FAFB62` |
  | Windows/MSVC | `animal_well.msh` | 8 | 6 | 72,075 | 1,306,205 | 0 | `65D7CC5BD515F12F3E8469234C8665D71FEA4E34354A70BD356954EC50F13902` |
  | Windows/MSVC | `animal_well.msh` | 4 | 3 | 144,150 | 1,691,928 | 0 | `18BCC55B25DBDE3A62976AC124A253E7F24BB35C5DD33E34DF1CC1E52545CCF2` |
  | Linux/GCC | `fluffy_ball.msh` | 8 | 6 | 28,878 | 455,919 | 0 | `9151C9EF90BB2F8463E821A6C9A348D09279CD13542FB0E542945F13574BBD57` |
  | Linux/GCC | `fluffy_ball.msh` | 4 | 3 | 57,755 | 623,480 | 0 | `473DCC303B0F33A3A47EDFF386F6B84E541EE03D94703EC56AF68EAD50EA30A6` |
  | Linux/GCC | `animal_well.msh` | 8 | 6 | 72,075 | 1,269,585 | 0 | `DA0EAB85097502138E670A8FD570765F814E3A2249D83B8D69B48EDDB4042A17` |
  | Linux/GCC | `animal_well.msh` | 4 | 3 | 144,150 | 1,694,400 | 0 | `6BB55CBC5830981D1D5CB13D91CFDAF7C48994F51DE41D4649AA2771357FAB26` |

  `fluffy_ball.msh` contains 173,263 vertices and 782,647 unique edges;
  `animal_well.msh` contains 432,450 vertices, 1,341,150 tetrahedra, and
  2,174,400 unique edges.
- A Linux/GCC run of the 13,328-part case under AddressSanitizer and
  UndefinedBehaviorSanitizer completed without diagnostics.
- A dedicated old/new sorting harness exercised all ten retained wrappers,
  including the currently unused `ikvsortii`, over six adversarial data
  patterns and 24 lengths from zero through 16,384. The 1,440 calls cover empty
  and singleton arrays, the 7/8/9 partition cutoff, power-of-two boundaries,
  random duplicate keys, all-equal keys, ascending/descending input, and
  integer extremes. Windows/MSVC and Linux/GCC both produced the same
  11,374,128-byte result with SHA-256
  `90B861E306F4482BB5CE268E4B97C1B5599FB628D9FB2BF6FDFB44F41381ADD8`.
