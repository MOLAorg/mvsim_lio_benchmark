"""Top-view plot of a world (terrain shading + primitive footprints) with its
trajectories overlaid. Saved as a small palette PNG."""
import io

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PatchCollection  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402
from PIL import Image  # noqa: E402

TRAJ_COLORS = ['#e6194b', '#1f4fe0', '#f58231', '#911eb4', '#008080', '#000000']


def plot_world(world, trajs, out_png, max_kb=300):
    xmin, xmax, ymin, ymax = world.extent
    span = max(xmax - xmin, ymax - ymin)
    aspect = (ymax - ymin) / (xmax - xmin)
    fig = plt.figure(figsize=(8, 8 * aspect + 2.6), dpi=100)
    gs = fig.add_gridspec(2, 1, height_ratios=[8 * aspect, 1.6])
    ax = fig.add_subplot(gs[0])
    axz = fig.add_subplot(gs[1])
    if world.terrain is not None:
        tz = world.terrain
        ls = matplotlib.colors.LightSource(azdeg=315, altdeg=45)
        rgb = ls.shade(tz.Z.T, cmap=plt.cm.gist_earth, vert_exag=2.0, blend_mode='soft',
                       dx=tz.res, dy=tz.res, vmin=tz.Z.min() - 0.3 * np.ptp(tz.Z) - 1,
                       vmax=tz.Z.max() + 0.6 * np.ptp(tz.Z) + 1)
        ax.imshow(rgb, origin='lower', extent=(tz.xmin, tz.xmax, tz.ymin, tz.ymax), zorder=0)
        cs = ax.contour(np.linspace(tz.xmin, tz.xmax, tz.nx), np.linspace(tz.ymin, tz.ymax, tz.ny),
                        tz.Z.T, levels=12, colors='k', linewidths=0.3, alpha=0.4, zorder=1)
        ax.clabel(cs, fontsize=6, fmt='%.0f m')
    # Footprints, low ones first so that tall structures stay visible:
    fps = sorted(world.footprints, key=lambda f: f.z_max)
    patches = []
    colors = []
    for fp in fps:
        if fp.kind == 'circle':
            patches.append(Circle(fp.data[:2], fp.data[2]))
        else:
            patches.append(Polygon(fp.data, closed=True))
        colors.append(fp.color)
    if patches:
        pc = PatchCollection(patches, facecolors=colors, edgecolors='#00000040', linewidths=0.2,
                             zorder=2)
        ax.add_collection(pc)
    for i, (name, tr) in enumerate(trajs.items()):
        c = TRAJ_COLORS[i % len(TRAJ_COLORS)]
        ax.plot(tr['x'], tr['y'], '-', color='white', lw=3.0, zorder=3, alpha=0.8)
        ax.plot(tr['x'], tr['y'], '-', color=c, lw=1.4, zorder=4, label=name)
        ax.plot(tr['x'][0], tr['y'][0], 'o', color=c, ms=7, mec='white', zorder=5)
        axz.plot(tr['t'] - tr['t'][0], tr['z'], '-', color=c, lw=1.2)
        k = len(tr['x']) // 3
        dx, dy = tr['x'][k + 20] - tr['x'][k], tr['y'][k + 20] - tr['y'][k]
        n = np.hypot(dx, dy)
        if n > 1e-6:
            ax.annotate('', xy=(tr['x'][k] + dx / n * span * 0.03, tr['y'][k] + dy / n * span * 0.03),
                        xytext=(tr['x'][k], tr['y'][k]),
                        arrowprops=dict(arrowstyle='-|>', color=c, lw=1.5), zorder=5)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{world.name}: top view and trajectories (o = start)')
    if trajs:
        ax.legend(loc='upper right', fontsize=8, framealpha=0.85)
    ax.grid(True, lw=0.3, alpha=0.5)
    axz.set_xlabel('time [s]')
    axz.set_ylabel('base_link z [m]')
    axz.grid(True, lw=0.3, alpha=0.5)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format='png')
    plt.close(fig)
    buf.seek(0)
    save_small_png(Image.open(buf), out_png, max_kb)


def save_small_png(img, out_png, max_kb=300):
    img = img.convert('RGB')
    for ncol in (256, 128, 64, 32):
        q = img.quantize(colors=ncol, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
        q.save(out_png, optimize=True)
        if out_png.stat().st_size <= max_kb * 1024:
            return
    scale = 0.8
    while out_png.stat().st_size > max_kb * 1024 and scale > 0.3:
        small = img.resize((int(img.width * scale), int(img.height * scale)), Image.LANCZOS)
        small.quantize(colors=64, method=Image.Quantize.MEDIANCUT,
                       dither=Image.Dither.NONE).save(out_png, optimize=True)
        scale -= 0.1
