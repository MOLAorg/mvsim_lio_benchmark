"""MVSim world XML builder restricted to the geometry that mvsim-dataset-gen
ray-traces exactly: boxes (vertical prisms, yaw only), cylinders, spheres,
ramps (wedges, full 6-DoF pose), horizontal planes, vertical walls with
openings, and one elevation map.

Every block class has <skip_elevation_adjust/> and <static>true</static>, so its
z is exactly the one given here (MVSim otherwise moves blocks onto the
terrain under them) and loading thousands of them stays cheap.
"""
import math
from dataclasses import dataclass, field

import numpy as np


def _n(v, nd=3):
    """Compact number formatting for the XML (no trailing zeros)."""
    s = f'{v:.{nd}f}'.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def _d(v):
    """Geometry dimensions are rounded to 1 cm (fewer distinct block classes)."""
    return round(float(v), 2)


@dataclass
class Footprint:
    """2D footprint of a primitive, for top-view plots and clearance checks."""
    kind: str  # 'poly' or 'circle'
    data: tuple
    z_min: float
    z_max: float
    color: str
    drivable: bool = False  # a surface to drive or walk on (ramps, steps)


@dataclass
class World:
    name: str
    description: str
    extent: tuple  # (xmin, xmax, ymin, ymax), for plots and screenshots
    classes: dict = field(default_factory=dict)
    blocks: list = field(default_factory=list)
    elements: list = field(default_factory=list)
    footprints: list = field(default_factory=list)
    hidden: list = field(default_factory=list)  # not drawn in top views (roofs, slabs)
    block_zmin: list = field(default_factory=list)  # lowest z of each block (cutaways)
    element_zmin: list = field(default_factory=list)
    terrain: object = None
    n_prims: int = 0

    # ------------------------------------------------------------------
    def _class(self, geom_xml, color):
        key = f'<color>{color}</color>{geom_xml}'
        if key not in self.classes:
            self.classes[key] = f'c{len(self.classes):04d}'
        return self.classes[key]

    def _block(self, geom_xml, pose, color, zmin):
        cls = self._class(geom_xml, color)
        x, y, z, yaw, pitch, roll = pose
        self.block_zmin.append(zmin)
        self.blocks.append(
            f'<block class="{cls}"><init_pose3d>{_n(x)} {_n(y)} {_n(z)} '
            f'{_n(math.degrees(yaw), 2)} {_n(math.degrees(pitch), 2)} {_n(math.degrees(roll), 2)}'
            f'</init_pose3d></block>\n')
        self.n_prims += 1

    # ------------------------------------------------------------------
    def box(self, cx, cy, z0, lx, ly, lz, yaw=0.0, color='#a0a0a0', drivable=False, plot=True):
        """Box centered at (cx, cy) in XY, from z0 to z0+lz, rotated `yaw` rad.
        MVSim's box origin is its corner, so it is shifted here."""
        lx, ly, lz = _d(lx), _d(ly), _d(lz)
        c, s = math.cos(yaw), math.sin(yaw)
        ox = cx - (c * lx / 2 - s * ly / 2)
        oy = cy - (s * lx / 2 + c * ly / 2)
        geom = f'<geometry type="box" lx="{lx}" ly="{ly}" lz="{lz}" />'
        self._block(geom, (ox, oy, z0, yaw, 0.0, 0.0), color, z0)
        corners = [(cx + c * dx - s * dy, cy + s * dx + c * dy)
                   for dx, dy in ((-lx / 2, -ly / 2), (lx / 2, -ly / 2), (lx / 2, ly / 2),
                                  (-lx / 2, ly / 2))]
        fp = Footprint('poly', tuple(corners), z0, z0 + lz, color, drivable)
        (self.footprints if plot else self.hidden).append(fp)

    def box_between(self, x0, y0, x1, y1, z0, width, lz, color='#a0a0a0'):
        """A wall-like box of thickness `width` along the segment (x0,y0)-(x1,y1)."""
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        self.box((x0 + x1) / 2, (y0 + y1) / 2, z0, length, width, lz, yaw, color)

    def cylinder(self, x, y, z0, radius, length, yaw=0.0, pitch=0.0, roll=0.0,
                 color='#806040'):
        """Cylinder whose axis starts at (x,y,z0) along the local +Z axis
        (vertical unless pitch/roll are given; pitch=pi/2 lays it along yaw)."""
        radius, length = _d(radius), _d(length)
        geom = (f'<geometry type="cylinder" radius="{radius}" '
                f'length="{length}" vertex_count="16" />')
        horiz = abs(pitch) > 1e-6 or abs(roll) > 1e-6
        self._block(geom, (x, y, z0, yaw, pitch, roll), color, z0 - radius if horiz else z0)
        if abs(pitch) < 1e-6 and abs(roll) < 1e-6:
            self.footprints.append(Footprint('circle', (x, y, radius), z0, z0 + length, color))
        else:
            # Horizontal-ish: approximate footprint by the axis' bounding rectangle.
            ax = (math.cos(yaw) * math.sin(pitch), math.sin(yaw) * math.sin(pitch))
            x1, y1 = x + ax[0] * length, y + ax[1] * length
            nx, ny = -ax[1], ax[0]
            nrm = math.hypot(nx, ny) or 1.0
            nx, ny = nx / nrm * radius, ny / nrm * radius
            self.footprints.append(Footprint(
                'poly', ((x + nx, y + ny), (x1 + nx, y1 + ny), (x1 - nx, y1 - ny),
                         (x - nx, y - ny)), z0 - radius, z0 + radius, color))

    def sphere(self, x, y, z, radius, color='#40a040'):
        radius = _d(radius)
        geom = f'<geometry type="sphere" radius="{radius}" vertex_count="12" />'
        self._block(geom, (x, y, z, 0.0, 0.0, 0.0), color, z - radius)
        self.footprints.append(Footprint('circle', (x, y, radius), z - radius, z + radius, color))

    def ramp(self, x, y, z, lx, ly, lz, yaw=0.0, pitch=0.0, roll=0.0, color='#909090',
             drivable=False, plot=True):
        """MVSim ramp (wedge): in its local frame it rises from z=0 at x=0 to
        z=lz at x=lx, across y in [-ly/2, ly/2]. Pose is its local origin."""
        lx, ly, lz = _d(lx), _d(ly), _d(lz)
        geom = f'<geometry type="ramp" lx="{lx}" ly="{ly}" lz="{lz}" />'
        inverted = abs(roll) > math.pi / 2
        z_lo, z_hi = (z - lz, z) if inverted else (z, z + lz)
        self._block(geom, (x, y, z, yaw, pitch, roll), color, z_lo)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x + c * dx - s * dy, y + s * dx + c * dy)
                   for dx, dy in ((0, -ly / 2), (lx, -ly / 2), (lx, ly / 2), (0, ly / 2))]
        fp = Footprint('poly', tuple(corners), z_lo, z_hi, color, drivable)
        (self.footprints if plot else self.hidden).append(fp)

    def hplane(self, xmin, xmax, ymin, ymax, z, color='#c0c0c0', plot=False):
        """Horizontal rectangle (zero thickness), e.g. floors or ceilings."""
        self.elements.append(
            f'\t<element class="horizontal_plane"><x_min>{xmin:.4f}</x_min><x_max>{xmax:.4f}'
            f'</x_max><y_min>{ymin:.4f}</y_min><y_max>{ymax:.4f}</y_max><z>{z:.4f}</z>'
            f'<color>{color}</color><cull_face>NONE</cull_face></element>\n')
        self.element_zmin.append(z)
        self.n_prims += 1
        if plot:
            self.footprints.append(Footprint(
                'poly', ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)), z, z, color))

    def wall(self, x0, y0, x1, y1, z0, height, thickness=0.2, openings=(), color='#d0d0c8'):
        """Vertical wall with optional openings: a list of dicts with keys
        kind ('door'|'window'), pos (0..1 along the wall), width, z_min, z_max
        (relative to the wall base). Openings are through-gaps for the LiDAR."""
        ops = ''.join(
            f'<{o["kind"]} position="{o["pos"]:.4f}" width="{o["width"]:.4f}" '
            f'z_min="{o["z_min"]:.4f}" z_max="{o["z_max"]:.4f}"/>' for o in openings)
        self.elements.append(
            f'\t<element class="vertical_plane"><x0>{x0:.4f}</x0><y0>{y0:.4f}</y0>'
            f'<x1>{x1:.4f}</x1><y1>{y1:.4f}</y1><z>{z0:.4f}</z><height>{height:.4f}</height>'
            f'<thickness>{thickness:.4f}</thickness><color>{color}</color>'
            f'<cull_face>NONE</cull_face>{ops}</element>\n')
        self.element_zmin.append(z0)
        self.n_prims += 1 + 2 * len(openings)
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 0:
            return
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        nx, ny = -uy * thickness / 2, ux * thickness / 2
        # Footprints of the solid parts: [a, b] spans along the wall with
        # their z range (relative to z0), mirroring how MVSim splits it.
        cuts = sorted({0.0, length} | {min(max(o['pos'] * length + d * o['width'] / 2, 0.0), length)
                                       for o in openings for d in (-1, 1)})
        for a, b in zip(cuts[:-1], cuts[1:]):
            if b - a < 1e-6:
                continue
            mid = (a + b) / 2
            op = next((o for o in openings
                       if abs(mid - o['pos'] * length) <= o['width'] / 2), None)
            spans = [(0.0, height)] if op is None else [(0.0, op['z_min']), (op['z_max'], height)]
            for zl, zh in spans:
                if zh - zl < 1e-4:
                    continue
                pa = (x0 + ux * a, y0 + uy * a)
                pb = (x0 + ux * b, y0 + uy * b)
                self.footprints.append(Footprint(
                    'poly', ((pa[0] + nx, pa[1] + ny), (pb[0] + nx, pb[1] + ny),
                             (pb[0] - nx, pb[1] - ny), (pa[0] - nx, pa[1] - ny)),
                    z0 + zl, z0 + zh, color))

    def set_terrain(self, terrain):
        self.terrain = terrain
        self.n_prims += 1

    # ------------------------------------------------------------------
    def to_xml(self, header_comment='', cut_z=None):
        """World XML. cut_z: drop everything whose lowest point is above it
        (cutaway renders of indoor worlds; never used for datasets)."""
        keep = (lambda zmin: True) if cut_z is None else (lambda zmin: zmin < cut_z)
        out = ['<mvsim_world version="1.0">\n']
        out.append('\t<!--\n')
        for line in (header_comment or self.description).strip().splitlines():
            out.append(f'\t  {line}\n')
        out.append('\t  Generated by tools/build_worlds.py: edit the world.py next to this\n'
                   '\t  file and rebuild, do not edit this XML by hand.\n\t-->\n')
        out.append('\t<simul_timestep>0.01</simul_timestep>\n')
        # Rendering-only settings (screenshots); ignored by mvsim-dataset-gen:
        out.append('\t<gui><ortho>false</ortho><show_forces>false</show_forces></gui>\n')
        out.append('\t<lights><enable_shadows>true</enable_shadows></lights>\n')
        if self.terrain is not None:
            out.append(self.terrain.to_xml())
        out.extend(e for e, zm in zip(self.elements, self.element_zmin) if keep(zm))
        for geom, cls in self.classes.items():
            out.append(f'<block:class name="{cls}"><static>true</static><skip_elevation_adjust/>'
                       f'{geom}</block:class>\n')
        out.extend(b for b, zm in zip(self.blocks, self.block_zmin) if keep(zm))
        # The sensor rig: chosen at generation time with the MVSIM_LIO_RIG
        # environment variable (path to a rigs/*.rig.xml file).
        out.append('\t<include file="$env{MVSIM_LIO_RIG}" />\n')
        out.append('</mvsim_world>\n')
        return ''.join(out)


class Terrain:
    """Elevation map sampled from an analytic, smooth z = f(x, y) function, so
    that trajectories can follow it exactly (no grid kinks under the wheels).
    Matrix rows run along X and columns along Y, as MVSim expects."""

    def __init__(self, f, xmin, xmax, ymin, ymax, resolution, color='#90a878'):
        self.f = f
        self.res = resolution
        self.nx = int(round((xmax - xmin) / resolution)) + 1
        self.ny = int(round((ymax - ymin) / resolution)) + 1
        self.xmin, self.ymin = xmin, ymin
        self.xmax = xmin + (self.nx - 1) * resolution
        self.ymax = ymin + (self.ny - 1) * resolution
        self.color = color
        xs = xmin + resolution * np.arange(self.nx)
        ys = ymin + resolution * np.arange(self.ny)
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        self.Z = np.asarray(f(X, Y), dtype=float)

    def height(self, x, y):
        return self.f(np.asarray(x, dtype=float), np.asarray(y, dtype=float))

    def to_xml(self):
        rows = ';\n'.join(' '.join(f'{v:.3f}' for v in row) for row in self.Z)
        return (f'\t<element class="elevation_map">\n\t\t<resolution>{self.res}</resolution>\n'
                f'\t\t<corner_min_x>{self.xmin}</corner_min_x>\n'
                f'\t\t<corner_min_y>{self.ymin}</corner_min_y>\n'
                f'\t\t<mesh_color>{self.color}</mesh_color>\n'
                f'\t\t<elevation_data_matrix>[\n{rows}\n]</elevation_data_matrix>\n'
                f'\t</element>\n')
