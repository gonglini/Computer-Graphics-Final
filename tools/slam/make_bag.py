"""Convert a simulator SLAM export (autopilot_sim --slamexport <dir>) into a ROS1 bag for FAST-LIO.

    python make_bag.py <export dir> [out.bag]

Topics
  /velodyne_points  sensor_msgs/PointCloud2  fields x y z intensity (float32), time (float32, s after
                    the header stamp), ring (uint16)   -> FAST-LIO preprocess "VELO16" (lidar_type 2)
  /imu/data         sensor_msgs/Imu  200 Hz, gyro rad/s, specific force m/s^2 (gravity included)
IMU and LiDAR share one frame (extrinsic = identity in the FAST-LIO config).
"""
import struct
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

T0 = 1_700_000_000.0   # simulation time 0 -> a plausible ROS time


def stamp(ts, t):
    t += T0
    sec = int(t)
    return ts.types['builtin_interfaces/msg/Time'](sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def main():
    src = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else src / 'sim_mapping.bag'
    if out.exists():
        out.unlink()
    ts = get_typestore(Stores.ROS1_NOETIC)
    Header = ts.types['std_msgs/msg/Header']
    PointField = ts.types['sensor_msgs/msg/PointField']
    PointCloud2 = ts.types['sensor_msgs/msg/PointCloud2']
    Imu = ts.types['sensor_msgs/msg/Imu']
    Quaternion = ts.types['geometry_msgs/msg/Quaternion']
    Vector3 = ts.types['geometry_msgs/msg/Vector3']

    # x y z intensity time (float32) + ring (uint16) = 22 bytes per point
    fields = [PointField(name='x', offset=0, datatype=7, count=1),
              PointField(name='y', offset=4, datatype=7, count=1),
              PointField(name='z', offset=8, datatype=7, count=1),
              PointField(name='intensity', offset=12, datatype=7, count=1),
              PointField(name='time', offset=16, datatype=7, count=1),
              PointField(name='ring', offset=20, datatype=4, count=1)]
    rec = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u2')])

    imu = np.loadtxt(src / 'imu.csv', delimiter=',', skiprows=1)
    with Writer(out) as w:
        c_pc = w.add_connection('/velodyne_points', PointCloud2.__msgtype__, typestore=ts)
        c_imu = w.add_connection('/imu/data', Imu.__msgtype__, typestore=ts)
        # IMU first (sorted by time together with the scans while writing)
        scans = []
        with open(src / 'scans.bin', 'rb') as f:
            while True:
                head = f.read(12)
                if len(head) < 12:
                    break
                t0, n = struct.unpack('<dI', head)
                scans.append((t0, f.tell(), n))
                f.seek(n * 22, 1)
        events = [(row[0], 'imu', row) for row in imu] + [(s[0] + 0.1, 'pc', s) for s in scans]
        events.sort(key=lambda e: e[0])
        cov = np.zeros(9)
        with open(src / 'scans.bin', 'rb') as f:
            for k, (t, kind, data) in enumerate(events):
                if kind == 'imu':
                    m = Imu(header=Header(seq=0, stamp=stamp(ts, t), frame_id='velodyne'),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=np.full(9, -1.0),
                            angular_velocity=Vector3(x=data[1], y=data[2], z=data[3]), angular_velocity_covariance=cov,
                            linear_acceleration=Vector3(x=data[4], y=data[5], z=data[6]),
                            linear_acceleration_covariance=cov)
                    w.write(c_imu, int((t + T0) * 1e9), ts.serialize_ros1(m, Imu.__msgtype__))
                else:
                    t0, off, n = data
                    f.seek(off)
                    pts = np.frombuffer(f.read(n * 22), dtype=rec)
                    m = PointCloud2(header=Header(seq=0, stamp=stamp(ts, t0), frame_id='velodyne'), height=1, width=n,
                                    fields=fields, is_bigendian=False, point_step=22, row_step=22 * n,
                                    data=np.frombuffer(pts.tobytes(), dtype=np.uint8), is_dense=True)
                    # stamp the message at the END of the sweep (when a driver would publish it)
                    w.write(c_pc, int((t0 + 0.1 + T0) * 1e9), ts.serialize_ros1(m, PointCloud2.__msgtype__))
                if k % 20000 == 0:
                    print(f'{k}/{len(events)}')
    print(f'wrote {out}  ({len(scans)} scans, {len(imu)} IMU samples, {out.stat().st_size / 1e6:.0f} MB)')


if __name__ == '__main__':
    main()
