"""Figures for the report from a FAST-LIO run on the simulator data (VTK offscreen + matplotlib).

    python render_results.py <export dir>
expects <export dir>/gt.csv and <export dir>/fastlio_result/{fastlio_map.pcd, odometry.csv}

Output (<export dir>/figures):
  map_top.png        FAST-LIO map from above, colored by height, estimated (red) + true (green) path
  map_3d.png         oblique 3D view of the map (viaduct, hills, city)
  map_detail_*.png   close-ups (downtown, viaduct)
  trajectory.png     estimated vs. ground-truth trajectory (top view)
  ate.png            absolute trajectory error over time
  stats.txt          ATE RMSE / mean / max, drift per distance
"""
import sys
from pathlib import Path

import numpy as np
import vtk
from vtk.util import numpy_support as ns

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if not line or line.startswith('#'):
                continue
            k, *v = line.split()
            header[k] = v
            if k == 'DATA':
                break
        fields = header['FIELDS']
        sizes = [int(s) for s in header['SIZE']]
        types = header['TYPE']
        n = int(header['POINTS'][0])
        dt = np.dtype([(name, {'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s)) for name, s, t in zip(fields, sizes, types)])
        if header['DATA'][0] == 'binary':
            arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        else:
            arr = np.loadtxt(f, dtype=dt, max_rows=n)
    return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64), \
        (arr['intensity'].astype(np.float64) if 'intensity' in fields else None)


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_odometry(path):
    import csv
    rows = list(csv.reader(open(path)))
    head = rows[0]
    idx = {h: i for i, h in enumerate(head)}
    t = np.array([float(r[idx['field.header.stamp']]) * 1e-9 for r in rows[1:]])
    p = np.array([[float(r[idx['field.pose.pose.position.' + a]]) for a in 'xyz'] for r in rows[1:]])
    return t - 1_700_000_000.0, p


def polyline(points, color, width):
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(points), deep=True))
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(len(points))
    for i in range(len(points)):
        lines.InsertCellPoint(i)
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetLines(lines)
    tube = vtk.vtkTubeFilter()
    tube.SetInputData(pd)
    tube.SetRadius(width)
    tube.SetNumberOfSides(8)
    m = vtk.vtkPolyDataMapper()
    m.SetInputConnection(tube.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(m)
    a.GetProperty().SetColor(*color)
    a.GetProperty().SetLighting(False)
    return a


def cloud_actor(xyz, zrange):
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(xyz), deep=True))
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    verts = vtk.vtkCellArray()
    ids = np.arange(len(xyz), dtype=np.int64)
    conn = np.empty((len(xyz), 2), dtype=np.int64)
    conn[:, 0] = 1
    conn[:, 1] = ids
    verts.SetCells(len(xyz), ns.numpy_to_vtkIdTypeArray(conn.ravel(), deep=True))
    pd.SetVerts(verts)
    z = ns.numpy_to_vtk(np.ascontiguousarray(xyz[:, 2]), deep=True)
    z.SetName('z')
    pd.GetPointData().SetScalars(z)
    lut = vtk.vtkLookupTable()
    lut.SetHueRange(0.66, 0.0)
    lut.SetRange(*zrange)
    lut.Build()
    m = vtk.vtkPolyDataMapper()
    m.SetInputData(pd)
    m.SetLookupTable(lut)
    m.SetScalarRange(*zrange)
    a = vtk.vtkActor()
    a.SetMapper(m)
    a.GetProperty().SetPointSize(1.6)
    return a, lut


def render(actors, cam_fn, path, size=(2200, 1500), title=None, lut=None):
    ren = vtk.vtkRenderer()
    ren.SetBackground(0.04, 0.05, 0.08)
    for a in actors:
        ren.AddActor(a)
    if title:
        t = vtk.vtkTextActor()
        t.SetInput(title)
        t.GetTextProperty().SetFontSize(30)
        t.GetTextProperty().SetColor(0.95, 0.97, 1.0)
        t.SetDisplayPosition(30, size[1] - 60)
        ren.AddViewProp(t)
    if lut is not None:
        bar = vtk.vtkScalarBarActor()
        bar.SetLookupTable(lut)
        bar.SetTitle('height [m]')
        bar.SetNumberOfLabels(5)
        bar.SetPosition(0.92, 0.1)
        bar.SetWidth(0.05)
        bar.SetHeight(0.6)
        ren.AddViewProp(bar)
    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.SetSize(*size)
    rw.AddRenderer(ren)
    cam_fn(ren.GetActiveCamera(), ren)
    rw.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(str(path))
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    print('wrote', path)


def main():
    src = Path(sys.argv[1])
    res = src / 'fastlio_result'
    out = src / 'figures'
    out.mkdir(exist_ok=True)

    gt = np.loadtxt(src / 'gt.csv', delimiter=',', skiprows=1)
    # FAST-LIO's world frame is the sensor frame at the start: express the ground truth in it
    R0 = quat_to_rot(gt[0, 4:8])
    p0 = gt[0, 1:4]
    gt_local = (gt[:, 1:4] - p0) @ R0
    t_od, p_od = load_odometry(res / 'odometry.csv')
    gt_at = np.stack([np.interp(t_od, gt[:, 0], gt_local[:, k]) for k in range(3)], axis=1)
    err = np.linalg.norm(p_od - gt_at, axis=1)
    dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(gt_at, axis=0), axis=1))])
    stats = (f'FAST-LIO on the LiDAR Drive simulator\n'
             f'trajectory length {dist[-1]:.0f} m, {len(t_od)} poses\n'
             f'ATE RMSE {np.sqrt(np.mean(err ** 2)):.3f} m, mean {err.mean():.3f} m, max {err.max():.3f} m\n'
             f'final position error {err[-1]:.3f} m  ({100 * err[-1] / max(1.0, dist[-1]):.3f} % of the distance)\n')
    (out / 'stats.txt').write_text(stats)
    print(stats)

    parts = sorted((res / 'pcd').glob('*.pcd')) or [res / 'fastlio_map.pcd']
    xyz = np.concatenate([read_pcd(p)[0] for p in parts])
    print(f'map: {len(parts)} pcd files, {len(xyz)} points')
    # thin out for rendering (the full map can be tens of millions of points)
    if len(xyz) > 6_000_000:
        xyz = xyz[np.random.default_rng(0).choice(len(xyz), 6_000_000, replace=False)]
    zr = (np.percentile(xyz[:, 2], 1), np.percentile(xyz[:, 2], 99))
    cloud, lut = cloud_actor(xyz, zr)
    path_est = polyline(p_od + [0, 0, 1.0], (1.0, 0.25, 0.2), 0.9)
    path_gt = polyline(gt_local[::20] + [0, 0, 1.3], (0.2, 1.0, 0.4), 0.6)
    c = xyz.mean(axis=0)
    ext = np.ptp(xyz[:, :2], axis=0).max()

    def top(cam, ren):
        cam.ParallelProjectionOn()
        cam.SetFocalPoint(*c)
        cam.SetPosition(c[0], c[1], c[2] + 2000)
        cam.SetViewUp(0, 1, 0)
        cam.SetParallelScale(0.55 * ext)
        ren.ResetCameraClippingRange()

    def oblique(cam, ren):
        cam.SetFocalPoint(*c)
        cam.SetPosition(c[0] - 0.55 * ext, c[1] - 0.75 * ext, c[2] + 0.55 * ext)
        cam.SetViewUp(0, 0, 1)
        cam.SetViewAngle(35)
        ren.ResetCameraClippingRange()

    render([cloud, path_est, path_gt], top, out / 'map_top.png',
           title='FAST-LIO map (red: estimated path, green: ground truth)', lut=lut)
    render([cloud, path_est], oblique, out / 'map_3d.png', title='FAST-LIO map - 3D view', lut=lut)
    # close-ups: start (downtown) and the highest point of the path (viaduct / mountain road)
    for name, p in (('start', gt_local[0]), ('high', gt_local[np.argmax(gt_local[:, 2])])):
        def close(cam, ren, p=p):
            cam.SetFocalPoint(*p)
            cam.SetPosition(p[0] - 90, p[1] - 120, p[2] + 80)
            cam.SetViewUp(0, 0, 1)
            cam.SetViewAngle(40)
            ren.ResetCameraClippingRange()
        render([cloud, path_est], close, out / f'map_detail_{name}.png', title=f'FAST-LIO map - {name}', lut=lut)

    plt.figure(figsize=(9, 8))
    plt.plot(gt_local[:, 0], gt_local[:, 1], 'g-', lw=2.5, label='ground truth')
    plt.plot(p_od[:, 0], p_od[:, 1], 'r--', lw=1.5, label='FAST-LIO')
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.title('Trajectory (top view)')
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / 'trajectory.png', dpi=150)
    plt.figure(figsize=(10, 4))
    plt.plot(dist, err, 'b-')
    plt.xlabel('distance driven [m]')
    plt.ylabel('position error [m]')
    plt.title('Absolute trajectory error (FAST-LIO vs. ground truth)')
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / 'ate.png', dpi=150)
    print('figures in', out)


if __name__ == '__main__':
    main()
