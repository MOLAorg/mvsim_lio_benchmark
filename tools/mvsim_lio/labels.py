"""Semantic classes of the benchmark worlds: the per-point "label" field of the
LiDAR sweeps rendered with `mvsim-dataset-gen --labels`. Outdoor classes reuse
the SemanticKITTI raw label ids; indoor ones start at 100. Every discrete
object (a car, a tree, a desk...) also has its own id in the "instance" field;
extended surfaces (roads, terrain, floors, walls...) have instance 0.
"""

CLASSES = {
    'unlabeled': 0,
    'outlier': 1,  # mixed returns between two surfaces (--mixed-return-label)
    'car': 10,
    'other-vehicle': 20,
    'road': 40,
    'parking': 44,
    'sidewalk': 48,
    'other-ground': 49,
    'building': 50,
    'fence': 51,
    'other-structure': 52,
    'vegetation': 70,
    'trunk': 71,
    'terrain': 72,
    'pole': 80,
    'traffic-sign': 81,
    'other-object': 99,
    'floor': 100,
    'ceiling': 101,
    'wall': 102,
    'column': 103,
    'stairs': 104,
    'table': 105,
    'chair': 106,
    'shelf': 107,
    'other-furniture': 108,
    'goods': 109,
}

# Mixed returns get this class (mvsim-dataset-gen --mixed-return-label).
OUTLIER = CLASSES['outlier']
