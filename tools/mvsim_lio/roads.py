"""Lane-following paths over a street graph."""
import math


def lane_path(nodes, offset):
    """Polyline through street-graph nodes, shifted `offset` m to the right of
    the driving direction (right-hand traffic). Corners are the intersections
    of consecutive shifted segments."""
    segs = []
    for (ax, ay), (bx, by) in zip(nodes[:-1], nodes[1:]):
        L = math.hypot(bx - ax, by - ay)
        nx, ny = (by - ay) / L, -(bx - ax) / L  # right-hand normal
        segs.append(((ax + nx * offset, ay + ny * offset), (bx + nx * offset, by + ny * offset)))
    pts = [segs[0][0]]
    for (p0, p1), (q0, q1) in zip(segs[:-1], segs[1:]):
        d1 = (p1[0] - p0[0], p1[1] - p0[1])
        d2 = (q1[0] - q0[0], q1[1] - q0[1])
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-9:
            pts.append(p1)
            continue
        t = ((q0[0] - p0[0]) * d2[1] - (q0[1] - p0[1]) * d2[0]) / den
        pts.append((p0[0] + t * d1[0], p0[1] + t * d1[1]))
    pts.append(segs[-1][1])
    return pts
