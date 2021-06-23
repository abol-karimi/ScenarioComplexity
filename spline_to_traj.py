from geomdl import BSpline
from generator import frame_to_distance, distance_to_pose


def spline_to_traj(degree, ctrlpts, knotvector, sample_size, sim_traj):
    curve = BSpline.Curve()
    curve.degree = degree
    curve.ctrlpts = ctrlpts
    curve.knotvector = knotvector
    curve.sample_size = sample_size
    frame2distance = [p[1] for p in curve.evalpts]
    frame2simDistance = frame_to_distance(sim_traj)
    traj = distance_to_pose(
        frame2distance, frame2simDistance, sim_traj)
    return traj


def curves_to_trajectories(curves, sim_trajs, sample_size):
    new_trajs = {}
    for car, curve in curves.items():
        degree = curve['degree']
        ctrlpts = curve['ctrlpts']
        knotvector = curve['knotvector']
        traj = spline_to_traj(degree, ctrlpts, knotvector,
                              sample_size, sim_trajs[car])
        new_trajs[car] = traj
    return new_trajs
