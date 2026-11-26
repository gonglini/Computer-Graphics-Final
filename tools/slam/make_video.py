"""Animation of the FAST-LIO map being built (VTK offscreen) -> MP4 + GIF for the report.

    python make_video.py <export dir> [seconds=60]

Every LiDAR scan of the export (sensor frame) is placed with FAST-LIO's estimated pose (/Odometry),
de-skewed with the motion during the 0.1 s sweep, voxel-downsampled (0.4 m). The video shows the map
growing (colored by height), the current scan (bright), the estimated path (red) and the vehicle,
with a chase camera, and ends with an overview of the whole map.
"""
import struct
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import vtk
from vtk.util import numpy_support as ns

T0 = 1_700_000_000.0
W, H = 1280, 720
FPS = 30
VOXEL = 0.4


def load_odometry(path):
    import csv
    rows = list(csv.reader(open(path)))
    i = {h: k for k, h in enumerate(rows[0])}
    t = np.array([float(r[i['field.header.stamp']]) * 1e-9 - T0 for r in rows[1:]])
    p = np.array([[float(r[i['field.pose.pose.position.' + a]]) for a in 'xyz'] for r in rows[1:]])
    q = np.array([[float(r[i['field.pose.pose.orientation.' + a]]) for a in 'xyzw'] for r in rows[1:]])
    return t, p, q


def quat_rot(q):
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z); R[..., 0, 1] = 2 * (x * y - z * w); R[..., 0, 2] = 2 * (x * z + y * w)
    R[..., 1, 0] = 2 * (x * y + z * w); R[..., 1, 1] = 1 - 2 * (x * x + z * z); R[..., 1, 2] = 2 * (y * z - x * w)
    R[..., 2, 0] = 2 * (x * z - y * w); R[..., 2, 1] = 2 * (y * z + x * w); R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def pose_at(t, ot, op, oq):
    """position + rotation at times t (linear / normalized quaternion interpolation)."""
    k = np.clip(np.searchsorted(ot, t) - 1, 0, len(ot) - 2)
    a = np.clip((t - ot[k]) / np.maximum(1e-9, ot[k + 1] - ot[k]), 0.0, 1.0)[:, None]
    p = op[k] * (1 - a) + op[k + 1] * a
    q0, q1 = oq[k], oq[k + 1].copy()
    q1[np.sum(q0 * q1, axis=1) < 0] *= -1
    q = q0 * (1 - a) + q1 * a
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return p, quat_rot(q)


def main():
    src = Path(sys.argv[1])
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    out = src / 'figures'
    out.mkdir(exist_ok=True)
    ot, op, oq = load_odometry(src / 'fastlio_result' / 'odometry.csv')

    # ---- every scan into the FAST-LIO map frame, voxel keys with the first scan that saw the voxel
    rec = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u2')])
    keys_all, pts_all, scan_of, scan_t, scan_pts = [], [], [], [], []
    with open(src / 'scans.bin', 'rb') as f:
        k = 0
        while True:
            head = f.read(12)
            if len(head) < 12:
                break
            t0, n = struct.unpack('<dI', head)
            a = np.frombuffer(f.read(n * 22), dtype=rec)
            if t0 + 0.1 < ot[0]:
                continue
            a = a[::2]                                   # half of the points is plenty for the picture
            t = t0 + a['time'].astype(np.float64)
            p, R = pose_at(t, ot, op, oq)
            xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
            w = np.einsum('nij,nj->ni', R, xyz) + p
            vk = np.floor(w / VOXEL).astype(np.int64)
            key = (vk[:, 0] + 20000) * 40000 * 4000 + (vk[:, 1] + 20000) * 4000 + (vk[:, 2] + 2000)
            keys_all.append(key)
            pts_all.append(w.astype(np.float32))
            scan_of.append(np.full(len(key), k, dtype=np.int32))
            scan_t.append(t0 + 0.1)
            scan_pts.append(w[::3].astype(np.float32))   # the current scan (bright) for this frame
            k += 1
            if k % 500 == 0:
                print('scan', k)
    keys = np.concatenate(keys_all)
    pts = np.concatenate(pts_all)
    sof = np.concatenate(scan_of)
    del keys_all, pts_all, scan_of
    _, first = np.unique(keys, return_index=True)          # first occurrence = the scan that discovered it
    first.sort()
    mpts = pts[first]
    msc = sof[first]
    order = np.argsort(msc, kind='stable')
    mpts = np.ascontiguousarray(mpts[order])
    msc = msc[order]
    nscan = len(scan_t)
    print(f'{nscan} scans, map {len(mpts)} voxels')

    # ---- VTK scene
    zlo, zhi = np.percentile(mpts[:, 2], 1), np.percentile(mpts[:, 2], 99)
    lut = vtk.vtkLookupTable()
    lut.SetHueRange(0.62, 0.0)
    lut.SetSaturationRange(0.85, 0.85)
    lut.SetRange(zlo, zhi)
    lut.Build()

    def cloud(points, size, color=None):
        pd = vtk.vtkPolyData()
        vp = vtk.vtkPoints()
        pd.SetPoints(vp)
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(pd)
        a = vtk.vtkActor()
        a.SetMapper(m)
        a.GetProperty().SetPointSize(size)
        if color:
            a.GetProperty().SetColor(*color)
            m.ScalarVisibilityOff()
        else:
            m.SetLookupTable(lut)
            m.SetScalarRange(zlo, zhi)
        return pd, a

    all_conn = np.arange(len(mpts), dtype=np.int64)
    all_off = np.arange(len(mpts) + 1, dtype=np.int64)
    map_pd, map_actor = cloud(mpts, 1.5)
    map_pd.GetPoints().SetData(ns.numpy_to_vtk(mpts, deep=False))
    zarr = ns.numpy_to_vtk(np.ascontiguousarray(mpts[:, 2]), deep=False)
    map_pd.GetPointData().SetScalars(zarr)
    verts = vtk.vtkCellArray()
    map_pd.SetVerts(verts)
    scan_pd, scan_actor = cloud(None, 2.5, (1.0, 1.0, 0.85))
    scan_actor.GetProperty().SetOpacity(0.9)

    path_pts = vtk.vtkPoints()
    path_lines = vtk.vtkCellArray()
    path_pd = vtk.vtkPolyData()
    path_pd.SetPoints(path_pts)
    path_pd.SetLines(path_lines)
    tube = vtk.vtkTubeFilter()
    tube.SetInputData(path_pd)
    tube.SetRadius(0.8)
    tube.SetNumberOfSides(8)
    pm = vtk.vtkPolyDataMapper()
    pm.SetInputConnection(tube.GetOutputPort())
    path_actor = vtk.vtkActor()
    path_actor.SetMapper(pm)
    path_actor.GetProperty().SetColor(1.0, 0.2, 0.15)
    path_actor.GetProperty().SetLighting(False)

    car = vtk.vtkCubeSource()
    car.SetXLength(4.5)
    car.SetYLength(1.9)
    car.SetZLength(1.6)
    cm = vtk.vtkPolyDataMapper()
    cm.SetInputConnection(car.GetOutputPort())
    car_actor = vtk.vtkActor()
    car_actor.SetMapper(cm)
    car_actor.GetProperty().SetColor(0.2, 0.9, 1.0)

    ren = vtk.vtkRenderer()
    ren.GradientBackgroundOn()
    ren.SetBackground(0.01, 0.015, 0.03)
    ren.SetBackground2(0.06, 0.08, 0.14)
    for a in (map_actor, scan_actor, path_actor, car_actor):
        ren.AddActor(a)
    title = vtk.vtkTextActor()
    title.GetTextProperty().SetFontSize(26)
    title.GetTextProperty().SetBold(True)
    title.GetTextProperty().SetColor(1, 1, 1)
    title.SetDisplayPosition(24, H - 46)
    title.SetInput('FAST-LIO SLAM  -  LiDAR Drive simulator (VLP-32C + IMU)')
    info = vtk.vtkTextActor()
    info.GetTextProperty().SetFontSize(18)
    info.GetTextProperty().SetColor(0.8, 0.88, 1.0)
    info.SetDisplayPosition(24, H - 78)
    for t in (title, info):
        ren.AddViewProp(t)
    bar = vtk.vtkScalarBarActor()
    bar.SetLookupTable(lut)
    bar.SetTitle('height [m]')
    bar.SetNumberOfLabels(4)
    bar.SetPosition(0.93, 0.12)
    bar.SetWidth(0.045)
    bar.SetHeight(0.5)
    ren.AddViewProp(bar)

    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.SetSize(W, H)
    rw.SetMultiSamples(4)
    rw.AddRenderer(ren)
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.ReadFrontBufferOff()

    cam = ren.GetActiveCamera()
    cam.SetViewAngle(50)
    pos_t = op[:, :]
    total_frames = int(seconds * FPS)
    build_frames = int(total_frames * 0.82)
    over_frames = total_frames - build_frames
    c_all = mpts.mean(axis=0)
    ext = np.ptp(mpts[:, :2], axis=0).max()

    mp4 = imageio.get_writer(out / 'fastlio_slam.mp4', fps=FPS, codec='libx264', quality=8, macro_block_size=8)
    gif_frames = []
    eye = None
    for fr in range(total_frames):
        if fr < build_frames:
            si = min(nscan - 1, int((fr + 1) / build_frames * nscan))
        else:
            si = nscan - 1
        n = int(np.searchsorted(msc, si, side='right'))
        verts.SetData(ns.numpy_to_vtkIdTypeArray(all_off[:n + 1], deep=True),
                      ns.numpy_to_vtkIdTypeArray(all_conn[:n], deep=True))
        map_pd.Modified()
        sp = scan_pts[si] if fr < build_frames else np.zeros((0, 3), np.float32)
        scan_pd.GetPoints().SetData(ns.numpy_to_vtk(np.ascontiguousarray(sp), deep=True))
        sv = vtk.vtkCellArray()
        if len(sp):
            sv.SetData(ns.numpy_to_vtkIdTypeArray(np.arange(len(sp) + 1, dtype=np.int64), deep=True),
                       ns.numpy_to_vtkIdTypeArray(np.arange(len(sp), dtype=np.int64), deep=True))
        scan_pd.SetVerts(sv)
        scan_pd.Modified()
        # path so far
        oi = int(np.searchsorted(ot, scan_t[si], side='right'))
        pp = op[:max(2, oi)] + np.array([0, 0, 0.8])
        path_pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(pp[::3]), deep=True))
        path_lines.Reset()
        m = len(pp[::3])
        path_lines.InsertNextCell(m)
        for i in range(m):
            path_lines.InsertCellPoint(i)
        path_pd.Modified()
        cur = op[min(len(op) - 1, oi)]
        R = quat_rot(oq[min(len(oq) - 1, oi)])
        fwd = R[:, 0]
        car_actor.SetPosition(*(cur + [0, 0, -0.6]))
        car_actor.SetOrientation(0, 0, np.degrees(np.arctan2(fwd[1], fwd[0])))
        car_actor.SetVisibility(fr < build_frames)

        dist = np.sum(np.linalg.norm(np.diff(op[:max(2, oi)], axis=0), axis=1))
        info.SetInput(f'scan {si + 1}/{nscan}   distance {dist / 1000:.2f} km   map {n / 1e6:.2f} M voxels ({VOXEL} m)')

        # camera: chase from behind and above, then fly out to the overview
        chase_eye = cur - 70 * fwd + np.array([0, 0, 55])
        chase_foc = cur + 25 * fwd
        if eye is None:
            eye, foc = chase_eye, chase_foc
        k = 0.06
        eye = eye + (chase_eye - eye) * k
        foc = foc + (chase_foc - foc) * k
        if fr >= build_frames:
            u = (fr - build_frames) / max(1, over_frames - 1)
            s = u * u * (3 - 2 * u)
            over_eye = c_all + np.array([0.0, -0.45 * ext, 0.95 * ext])
            e2, f2 = eye + (over_eye - eye) * s, foc + (c_all - foc) * s
        else:
            e2, f2 = eye, foc
        cam.SetPosition(*e2)
        cam.SetFocalPoint(*f2)
        cam.SetViewUp(0, 0, 1)
        ren.ResetCameraClippingRange()
        rw.Render()
        w2i.Modified()
        w2i.Update()
        img = w2i.GetOutput()
        arr = ns.vtk_to_numpy(img.GetPointData().GetScalars()).reshape(H, W, -1)[::-1, :, :3]
        mp4.append_data(arr)
        if fr % 3 == 0:
            gif_frames.append(arr[::2, ::2].copy())
        if fr % 150 == 0:
            print('frame', fr, '/', total_frames)
    mp4.close()
    # GIF: 640x360, 10 fps
    imageio.mimsave(out / 'fastlio_slam.gif', gif_frames, duration=0.1, loop=0)
    print('wrote', out / 'fastlio_slam.mp4', 'and', out / 'fastlio_slam.gif')


if __name__ == '__main__':
    main()
