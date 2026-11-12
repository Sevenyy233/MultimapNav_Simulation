"""Conservative primitive coverage in the stowed base frame."""
from itertools import product
import numpy as np
from scipy.spatial.transform import Rotation

SIGNS = np.array(list(product([-1., 1.], repeat=3)))
EDGES = [(i, j) for i in range(8) for j in range(i) if np.count_nonzero(SIGNS[i]-SIGNS[j]) == 1]


def primitive_corners(transform, half):
    return (SIGNS*half)@transform[:3, :3].T+transform[:3, 3]


def fit_boxes(name, primitives, link_rotation, padding=.025):
    """Every clipped box contains its primitive volumes, not just sampled faces."""
    vertices = np.concatenate(primitives)
    if name == 'base_link' or 'track_belt' in name or name.startswith('rslidar_'):
        rotation = link_rotation
    elif name == 'link7':
        # The bucket's link axes are CAD axes, not its horizontal mounting axes.
        rotation = np.eye(3)
    else:
        _, vectors = np.linalg.eigh(np.cov(vertices.T))
        if np.linalg.det(vectors) < 0:
            vectors[:, 0] *= -1
        candidates = [np.eye(3), link_rotation, vectors]
        rotation = min(candidates, key=lambda r: np.prod(np.ptp(vertices@r, axis=0)))
    local = [v@rotation for v in primitives]
    all_points = np.concatenate(local)
    lower, upper = all_points.min(axis=0), all_points.max(axis=0)
    # Small-arm fittings and hydraulic parts should not fill one large empty
    # diagonal rectangle. Clip the exact convex primitive boxes into four slabs.
    axis = int(np.argmax(upper-lower))
    cuts = np.linspace(lower[axis], upper[axis], 5 if name == 'link4' else 2)
    boxes = []
    for k, (lo, hi) in enumerate(zip(cuts[:-1], cuts[1:])):
        clipped = []
        for points in local:
            clipped.extend(points[(points[:, axis] >= lo-1e-9) & (points[:, axis] <= hi+1e-9)])
            for i, j in EDGES:
                delta = points[j, axis]-points[i, axis]
                if abs(delta) < 1e-12:
                    continue
                for boundary in (lo, hi):
                    t = (boundary-points[i, axis])/delta
                    if 0 <= t <= 1:
                        clipped.append(points[i]+t*(points[j]-points[i]))
        if not clipped:
            continue
        points = np.array(clipped)
        a, b = points.min(axis=0)-padding, points.max(axis=0)+padding
        boxes.append({'name': f'{name}_{k}' if name == 'link4' else name,
                      'source_link': name, 'center': (rotation@((a+b)/2)).tolist(),
                      'size': (b-a).tolist(),
                      'quaternion': Rotation.from_matrix(rotation).as_quat().tolist()})
    return boxes
