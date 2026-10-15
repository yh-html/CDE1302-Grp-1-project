"""Helpers shared by the CDE1302 frame build scripts that run inside Fusion.

Every length or angle goes in as a Fusion expression string ('3 mm', 'arm_r + 2 mm',
'cam_tilt_min'). Numbers are only used to seed geometry near its final place; the
driving sketch dimensions and feature inputs carry the expressions, so changing a
User Parameter regenerates the part. Fusion's internal length unit is cm.

Modelling frame: Z up, +X forward, +Y left (frame plane = XY).
"""
import math

import adsk.core
import adsk.fusion

H = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
V = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
A = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

REF_HALF = 30.0  # cm, half-length of the sketch reference axes (covers any part here)


def design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def ev(expr, unit='mm'):
    """Evaluate an expression to internal units (cm for lengths, rad for angles)."""
    return design().unitsManager.evaluateExpression(expr, unit)


def vi(expr):
    return adsk.core.ValueInput.createByString(expr)


def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def add_param(name, expr, unit, comment):
    ups = design().userParameters
    if ups.itemByName(name):
        raise ValueError(f'parameter {name} already exists')
    return ups.add(name, vi(expr), unit, comment)


def comp(name):
    """(component, occurrence) of a top-level component, by name."""
    for occ in design().rootComponent.occurrences:
        if occ.component.name == name:
            return occ.component, occ
    raise KeyError(name)


def new_comp(name):
    occ = design().rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component, occ


def offset_plane(c, expr, name):
    """Construction plane parallel to XY at height expr."""
    inp = c.constructionPlanes.createInput()
    inp.setByOffset(c.xYConstructionPlane, vi(expr))
    plane = c.constructionPlanes.add(inp)
    plane.name = name
    return plane


def body(c, name):
    b = c.bRepBodies.itemByName(name)
    if not b:
        raise KeyError(name)
    return b


def collection(items):
    col = adsk.core.ObjectCollection.create()
    for item in items:
        col.add(item)
    return col


def extrude(c, profiles, dist, op, name, bodies=None):
    """Extrude profiles by the expression dist (positive = sketch normal)."""
    inp = c.features.extrudeFeatures.createInput(profiles, op)
    inp.setDistanceExtent(False, vi(dist))
    if bodies:
        inp.participantBodies = bodies
    feat = c.features.extrudeFeatures.add(inp)
    feat.name = name
    return feat


def extrude_sym(c, profiles, full, op, name, bodies=None):
    """Extrude symmetrically about the sketch plane, total length full."""
    inp = c.features.extrudeFeatures.createInput(profiles, op)
    inp.setSymmetricExtent(vi(full), True)
    if bodies:
        inp.participantBodies = bodies
    feat = c.features.extrudeFeatures.add(inp)
    feat.name = name
    return feat


def cut_through(c, profiles, name, bodies=None, symmetric=False, dist=None):
    """Cut through the part on the positive side of the sketch plane (or both sides).

    Prefer dist (an expression): through-all extents keep references to every body in the
    design and go stale when another component is deleted. Both sides without dist uses two
    through-all extents: setAllExtent(SymmetricExtentDirection) only cut one side.
    """
    inp = c.features.extrudeFeatures.createInput(profiles, CUT)
    if dist and symmetric:
        inp.setSymmetricExtent(vi(dist), True)
    elif dist:
        inp.setDistanceExtent(False, vi(dist))
    elif symmetric:
        inp.setTwoSidesExtent(adsk.fusion.ThroughAllExtentDefinition.create(),
                              adsk.fusion.ThroughAllExtentDefinition.create(), vi('0 deg'), vi('0 deg'))
    else:
        inp.setAllExtent(adsk.fusion.ExtentDirections.PositiveExtentDirection)
    if bodies:
        inp.participantBodies = bodies
    feat = c.features.extrudeFeatures.add(inp)
    feat.name = name
    return feat


def pattern(c, feats, axis, qty, angle, name):
    """Circular pattern of features or bodies; qty and angle are expressions."""
    inp = c.features.circularPatternFeatures.createInput(collection(feats), axis)
    inp.quantity = vi(qty)
    inp.totalAngle = vi(angle)
    inp.isSymmetric = False
    feat = c.features.circularPatternFeatures.add(inp)
    feat.name = name
    return feat


def rect_pattern(c, feats, direction, qty, spacing, name):
    """Rectangular pattern along one direction; qty and spacing are expressions."""
    inp = c.features.rectangularPatternFeatures.createInput(
        collection(feats), direction, vi(qty), vi(spacing),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    feat = c.features.rectangularPatternFeatures.add(inp)
    feat.name = name
    return feat


def mirror(c, feats, plane, name):
    feat = c.features.mirrorFeatures.add(c.features.mirrorFeatures.createInput(collection(feats), plane))
    feat.name = name
    return feat


def z_axis(c):
    """A real construction axis on the component Z axis (move features reject the origin axis)."""
    axis = c.constructionAxes.itemByName('ZAxis')
    if axis:
        return axis
    inp = c.constructionAxes.createInput()
    inp.setByTwoPlanes(c.xZConstructionPlane, c.yZConstructionPlane)
    axis = c.constructionAxes.add(inp)
    axis.name = 'ZAxis'
    return axis


def rotate(c, occ, bodies, axis, angle, name):
    """Parametric move: rotate bodies about axis by the expression angle.

    Move features in a sub-component only accept assembly-context proxies.
    """
    inp = c.features.moveFeatures.createInput2(collection([b.createForAssemblyContext(occ) for b in bodies]))
    inp.defineAsRotate(axis.createForAssemblyContext(occ), vi(angle))
    feat = c.features.moveFeatures.add(inp)
    feat.name = name
    return feat


def combine(c, target, tools, name, op=JOIN):
    inp = c.features.combineFeatures.createInput(target, collection(tools))
    inp.operation = op
    feat = c.features.combineFeatures.add(inp)
    feat.name = name
    return feat


def fillet(c, edges, radius, name):
    inp = c.features.filletFeatures.createInput()
    inp.edgeSetInputs.addConstantRadiusEdgeSet(collection(edges), vi(radius), False)
    feat = c.features.filletFeatures.add(inp)
    feat.name = name
    return feat


def shell(c, faces, thickness, name):
    """Shell the owning body, removing faces, with the expression wall thickness."""
    inp = c.features.shellFeatures.createInput(collection(faces), False)
    inp.insideThickness = vi(thickness)
    feat = c.features.shellFeatures.add(inp)
    feat.name = name
    return feat


def snapshot(path, only=None, view='IsoTopRightViewOrientation', w=1400, h=900):
    """Save a fitted view to path; with only=[component names], hide the other top-level occurrences."""
    occs = list(design().rootComponent.occurrences)
    was = {o.name: o.isLightBulbOn for o in occs}
    if only:
        for o in occs:
            o.isLightBulbOn = o.component.name in only
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.viewOrientation = getattr(adsk.core.ViewOrientations, view)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    vp.saveAsImageFile(path, w, h)
    for o in occs:
        o.isLightBulbOn = was[o.name]


def mm(v):
    """Internal cm -> mm, rounded for reports."""
    return round(v * 10, 2)


def bbox_mm(b):
    bb = b.boundingBox
    return [mm(v) for v in bb.minPoint.asArray()], [mm(v) for v in bb.maxPoint.asArray()]


class Sk:
    """Sketch whose geometry is placed by expression-driven dimensions from the origin.

    xl / yl are construction lines through the origin along sketch x / y; a coordinate
    given as '0' is held by a coincident constraint to the matching line instead of a
    zero-length dimension.
    """

    def __init__(self, c, plane, name):
        self.s = c.sketches.add(plane)
        self.s.name = name
        self.dims = self.s.sketchDimensions
        self.gc = self.s.geometricConstraints
        self.o = self.s.originPoint
        self.xl = self._ref_line(pt(-REF_HALF, 0), pt(REF_HALF, 0), self.gc.addHorizontal)
        self.yl = self._ref_line(pt(0, -REF_HALF), pt(0, REF_HALF), self.gc.addVertical)

    def _ref_line(self, a, b, orient):
        line = self.s.sketchCurves.sketchLines.addByTwoPoints(a, b)
        line.isConstruction = True
        orient(line)
        self.gc.addCoincident(self.o, line)
        line.startSketchPoint.isFixed = True
        line.endSketchPoint.isFixed = True
        return line

    def place(self, p, x, y):
        """Hold sketch point p at signed expressions (x, y)."""
        for expr, axis_line, orient in ((x, self.yl, H), (y, self.xl, V)):
            if expr == '0':
                self.gc.addCoincident(p, axis_line)
                continue
            g = p.geometry
            d = self.dims.addDistanceDimension(self.o, p, orient, pt(g.x / 2 + 0.3, g.y / 2 + 0.3))
            d.parameter.expression = expr if ev(expr) > 0 else f'-({expr})'

    def point(self, x, y):
        p = self.s.sketchPoints.add(pt(ev(x), ev(y)))
        self.place(p, x, y)
        return p

    def circle(self, x, y, d):
        """Circle of diameter d centred at (x, y)."""
        circles = self.s.sketchCurves.sketchCircles
        if x == '0' and y == '0':
            c = circles.addByCenterRadius(self.o, ev(d) / 2)
        else:
            c = circles.addByCenterRadius(pt(ev(x), ev(y)), ev(d) / 2)
            self.place(c.centerSketchPoint, x, y)
        return self._dia(c, d)

    def poly(self, pts):
        """Closed polygon through expression vertices [(x, y), ...]."""
        sp = [self.point(x, y) for x, y in pts]
        lines = self.s.sketchCurves.sketchLines
        return [lines.addByTwoPoints(sp[i], sp[(i + 1) % len(sp)]) for i in range(len(sp))]

    def circle_at(self, p, d):
        """Circle of diameter d centred on an existing sketch point."""
        return self._dia(self.s.sketchCurves.sketchCircles.addByCenterRadius(p, ev(d) / 2), d)

    def _dia(self, c, d):
        g = c.centerSketchPoint.geometry
        dim = self.dims.addDiameterDimension(c, pt(g.x + ev(d), g.y + ev(d)))
        dim.parameter.expression = d
        return c

    def rect(self, x0, y0, w, h):
        """Axis-aligned rectangle: lower-left corner (x0, y0), width w, height h."""
        X0, Y0 = ev(x0), ev(y0)
        lines = self.s.sketchCurves.sketchLines.addTwoPointRectangle(pt(X0, Y0), pt(X0 + ev(w), Y0 + ev(h)))
        aligned = {adsk.fusion.HorizontalConstraint.classType(), adsk.fusion.VerticalConstraint.classType()}
        corners = {}
        for line in lines:
            if not {gc.objectType for gc in line.geometricConstraints} & aligned:
                a, b = line.startSketchPoint.geometry, line.endSketchPoint.geometry
                (self.gc.addHorizontal if abs(a.y - b.y) < 1e-9 else self.gc.addVertical)(line)
            for p in (line.startSketchPoint, line.endSketchPoint):
                corners[(round(p.geometry.x, 6), round(p.geometry.y, 6))] = p
        ll, ur = corners[min(corners)], corners[max(corners)]
        self.dims.addDistanceDimension(ll, ur, H, pt(X0 + ev(w) / 2, Y0 - 0.3)).parameter.expression = w
        self.dims.addDistanceDimension(ll, ur, V, pt(X0 - 0.3, Y0 + ev(h) / 2)).parameter.expression = h
        self.place(ll, x0, y0)
        return lines

    def polar(self, centre, r, ang, cw=False):
        """Point at radius r, angle ang (0 < ang < 180; from +x, CCW, or CW if cw) about centre on xl.

        In an XZ-plane sketch, sketch y is model -Z, so a model angle measured from +X toward +Z
        is a clockwise sketch angle: pass cw=True.
        """
        return self.polar_line(centre, r, ang, cw)[0]

    def polar_line(self, centre, r, ang, cw=False):
        """As polar(), also returning the construction line centre -> point."""
        c = centre.geometry
        R, a = ev(r), ev(ang, 'deg') * (-1 if cw else 1)
        line = self.s.sketchCurves.sketchLines.addByTwoPoints(
            centre, pt(c.x + R * math.cos(a), c.y + R * math.sin(a)))
        line.isConstruction = True
        self.dims.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint, A,
            pt(c.x + R * math.cos(a) / 2 + 0.2, c.y + R * math.sin(a) / 2)).parameter.expression = r
        # The angle is measured where the line meets xl, in the quadrant holding the text point.
        ix = c.x - c.y * math.cos(a) / math.sin(a) if abs(math.sin(a)) > 1e-9 else c.x
        text = pt(ix + R / 2 * math.cos(a / 2), R / 2 * math.sin(a / 2))
        self.dims.addAngularDimension(self.xl, line, text).parameter.expression = ang
        return line.endSketchPoint, line

    def bar(self, centre, r0, r1, ang, w0, w1=None):
        """Quadrilateral on the ray from centre at ang: width w0 at radius r0, w1 at r1."""
        w1 = w1 or w0
        pa, _ = self.polar_line(centre, r0, ang)
        pb, axis = self.polar_line(centre, r1, ang)
        a = ev(ang, 'deg')
        nx, ny = -math.sin(a), math.cos(a)
        ga, gb = pa.geometry, pb.geometry
        h0, h1 = ev(w0) / 2, ev(w1) / 2
        lines = self.s.sketchCurves.sketchLines
        l1 = lines.addByTwoPoints(pt(ga.x + nx * h0, ga.y + ny * h0), pt(gb.x + nx * h1, gb.y + ny * h1))
        l2 = lines.addByTwoPoints(l1.endSketchPoint, pt(gb.x - nx * h1, gb.y - ny * h1))
        l3 = lines.addByTwoPoints(l2.endSketchPoint, pt(ga.x - nx * h0, ga.y - ny * h0))
        l4 = lines.addByTwoPoints(l3.endSketchPoint, l1.startSketchPoint)
        self.gc.addMidPoint(pa, l4)
        self.gc.addMidPoint(pb, l2)
        self.gc.addPerpendicular(l4, axis)
        self.gc.addPerpendicular(l2, axis)
        for line, w, g in ((l4, w0, ga), (l2, w1, gb)):
            self.dims.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, A,
                                           pt(g.x + 0.3, g.y + 0.3)).parameter.expression = w
        return pa, pb

    def bar2(self, pa, pb, w):
        """Rectangle of width w centred on the segment between sketch points pa and pb."""
        ga, gb = pa.geometry, pb.geometry
        axis = self.s.sketchCurves.sketchLines.addByTwoPoints(pa, pb)
        axis.isConstruction = True
        L = math.hypot(gb.x - ga.x, gb.y - ga.y)
        nx, ny = -(gb.y - ga.y) / L, (gb.x - ga.x) / L
        h = ev(w) / 2
        lines = self.s.sketchCurves.sketchLines
        l1 = lines.addByTwoPoints(pt(ga.x + nx * h, ga.y + ny * h), pt(gb.x + nx * h, gb.y + ny * h))
        l2 = lines.addByTwoPoints(l1.endSketchPoint, pt(gb.x - nx * h, gb.y - ny * h))
        l3 = lines.addByTwoPoints(l2.endSketchPoint, pt(ga.x - nx * h, ga.y - ny * h))
        l4 = lines.addByTwoPoints(l3.endSketchPoint, l1.startSketchPoint)
        self.gc.addMidPoint(pa, l4)
        self.gc.addMidPoint(pb, l2)
        self.gc.addPerpendicular(l4, axis)
        self.gc.addPerpendicular(l2, axis)
        self.gc.addParallel(l1, axis)
        self.dims.addDistanceDimension(l4.startSketchPoint, l4.endSketchPoint, A,
                                       pt(ga.x + 0.3, ga.y + 0.3)).parameter.expression = w

    def slot(self, centre, r0, r1, ang, w):
        """Stadium slot of width w along the ray from centre at ang, end centres at r0 and r1."""
        pa, pb = self.bar(centre, r0, r1, ang, w)
        self.circle_at(pa, w)
        self.circle_at(pb, w)

    def rect_c(self, cx, cy, w, h):
        """Axis-aligned rectangle centred at (cx, cy)."""
        x0 = f'-({w}) / 2' if cx == '0' else f'({cx}) - ({w}) / 2'
        y0 = f'-({h}) / 2' if cy == '0' else f'({cy}) - ({h}) / 2'
        return self.rect(x0, y0, w, h)

    def profiles(self):
        return collection(self.s.profiles)
