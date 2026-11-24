# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
"""只构造合成平面和孤点，演示去噪候选；不读取设备和地图，不写文件。"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import numpy as np
from desktop.repair_core import apply_operation

axis = np.linspace(-1, 1, 31)
x, y = np.meshgrid(axis, axis)
plane = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))).astype('f4')
points = np.vstack((plane, [[6, 6, 6], [-8, 4, 3]])).astype('f4')
result = apply_operation(points, 'radius', {'radius': 0.11, 'min_neighbors': 3})
filtered = result.points[result.keep]
assert len(filtered) == len(plane), result.stats
np.testing.assert_array_equal(filtered, plane)
print(f'合成点云：{len(points)} -> {len(filtered)}，移除2个孤点，原平面保留。')
