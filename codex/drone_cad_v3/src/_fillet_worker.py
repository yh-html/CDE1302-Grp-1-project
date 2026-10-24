"""Run one OCC fillet/chamfer in an isolated process (OCC can segfault on bad edge sets)."""
import sys, json
import cadquery as cq
inp, out, mode, r, keys = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), json.loads(sys.argv[5])
S = cq.Shape.importBrep(inp)
es = [e for e in S.Edges() if any(abs(e.Center().x - k[0]) < .011 and abs(e.Center().y - k[1]) < .011 and abs(e.Center().z - k[2]) < .011 for k in keys)]
if len(es) != len(keys): sys.exit(3)
F = S.fillet(r, es) if mode == 'fillet' else S.chamfer(r, None, es)
if not F.isValid() or len(F.Solids()) != 1: sys.exit(4)
F.exportBrep(out)
