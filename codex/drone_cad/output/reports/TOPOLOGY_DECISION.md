# Topology decision

Selected **fork**: paired splayed lower chords, a single curved upper chord and two branching webs.

All three candidates were built as low-detail B-rep solids and rendered before the detailed assembly. Scores are ordinal engineering judgment (5 best), not FEA or measured strength.

| Candidate | Arm volume mm3 | Mass tendency | Crash load path | Printability | Interface risk (5 low) | Visual fit | Total |
|---|---:|---:|---:|---:|---:|---:|---:|
| arch | 1809.8 | 5 | 2 | 4 | 4 | 3 | 18 |
| fork | 2572.7 | 4 | 5 | 3 | 4 | 5 | 21 |
| cage | 3914.4 | 2 | 4 | 2 | 3 | 4 | 15 |

Arch has the least material but lacks the forked torsional load path. Cage adds tip material and crowded supports. Fork spreads the lower root in plan, keeps visible deep lens cells and preserves a single unobstructed electronics well. Adjacent-arm ties close a torsional path. No load test or buckling analysis is implied.
