# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 中文源码导读｜验证与基准
# 职责：按空间邻居阈值清理孤点。
# 此文件用于构造场景、运行断言或生成评估记录，不是日常软件启动入口。
# 运行范围由本文件参数、mock及主入口决定；手动验证可能创建输出、编译内核或打开自建窗口。
"""Point-radius denoising using this workstation's exported PLY format."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from desktop.white_cloud_filter import clean_source, export_clean
from desktop.white_mesh_io import iter_exported_points


# 验证分组 CleanCloudTests：按空间邻居阈值清理孤点。
# 本组组织的用例/夹具入口：test_inclusive_radius_excludes_self_and_keeps_source_intact、test_cancellation_cleans_staging_and_source_map_is_protected。
class CleanCloudTests(unittest.TestCase):
    # 回归场景 test_inclusive_radius_excludes_self_and_keeps_source_intact：围绕“按空间邻居阈值清理孤点”检查下面列出的输入/状态与结果约束。
    # 验收：(counts['input_points'], counts['kept_points'], counts['removed_points'], counts['invalid_points']) 应与 (5, 3, 1, 1) 相等。
    # 验收：removed.tolist() 应与 [[10, 10, 0]] 相等。
    # 验收：strict['kept_points'] 应与 1 相等。
    def test_inclusive_radius_excludes_self_and_keeps_source_intact(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            p = d/'input.ply'
            p.write_bytes(b'ply\nformat binary_little_endian 1.0\nelement vertex 5\nproperty float x\n'
                          b'property float y\nproperty float z\nend_header\n' +
                          np.array([[0,0,0],[.125,0,0],[0,.125,0],[10,10,0],[np.nan,0,0]], dtype='<f4').tobytes())
            source = p.read_bytes()
            xyz, removed, counts = clean_source(p, 12.5, 1)
            # 验收：(counts['input_points'], counts['kept_points'], counts['removed_points'], counts['invalid_points']) 应与 (5, 3, 1, 1) 相等。
            self.assertEqual((counts['input_points'], counts['kept_points'], counts['removed_points'], counts['invalid_points']), (5,3,1,1))
            # 验收：removed.tolist() 应与 [[10, 10, 0]] 相等。
            self.assertEqual(removed.tolist(), [[10,10,0]])
            _, _, strict = clean_source(p, 12.5, 2)
            # 验收：strict['kept_points'] 应与 1 相等。
            self.assertEqual(strict['kept_points'], 1)
            _, _, off = clean_source(p, 12.5, 0)
            # 验收：off['kept_points'] 应与 4 相等。
            self.assertEqual(off['kept_points'], 4)
            out = export_clean(d,p,xyz,counts)
            # 验收：np.vstack(list(iter_exported_points(out / 'cleaned.ply'))) 应与 xyz 相等。
            np.testing.assert_array_equal(np.vstack(list(iter_exported_points(out/'cleaned.ply'))), xyz)
            # 验收：json.loads((out / 'filter.json').read_text('utf8'))['removed_points'] 应与 1 相等。
            self.assertEqual(json.loads((out/'filter.json').read_text('utf8'))['removed_points'], 1)
            # 验收：out 不应等于 export_clean(d, p, xyz, counts)。
            self.assertNotEqual(out, export_clean(d,p,xyz,counts))
            # 验收：p.read_bytes() 应与 source 相等。
            self.assertEqual(p.read_bytes(), source)

    # 回归场景 test_cancellation_cleans_staging_and_source_map_is_protected：围绕“按空间邻居阈值清理孤点”检查下面列出的输入/状态与结果约束。
    # 验收：list(d.iterdir()) 应与 [p] 相等。
    # 异常边界：该路径预期抛出 ValueError，同时匹配 '源地图'。
    # 异常边界：该路径预期抛出 InterruptedError。
    def test_cancellation_cleans_staging_and_source_map_is_protected(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);p=d/'protected.map';p.mkdir()
            stop=threading.Event()
            # 异常边界：该路径预期抛出 ValueError，同时匹配 '源地图'。
            with self.assertRaisesRegex(ValueError,'源地图'):
                export_clean(p,p,np.zeros((3,3)),{})
            # 验证分组 Cancel：按空间邻居阈值清理孤点。
            # 本组组织的用例/夹具入口：__init__、is_set。
            class Cancel:
                # 测试辅助入口 __init__：服务于“按空间邻居阈值清理孤点”场景。
                def __init__(self):self.checks=0
                # 测试辅助入口 is_set：服务于“按空间邻居阈值清理孤点”场景。
                # 返回路径：self.checks >= 3。
                def is_set(self):
                    self.checks+=1
                    return self.checks>=3
            # 异常边界：该路径预期抛出 InterruptedError。
            with self.assertRaises(InterruptedError):
                export_clean(d,p,np.zeros((3,3)),{},Cancel())
            # 验收：list(d.iterdir()) 应与 [p] 相等。
            self.assertEqual(list(d.iterdir()),[p])
            stop.set()
            # 异常边界：该路径预期抛出 InterruptedError。
            with self.assertRaises(InterruptedError):
                clean_source(p,15,3,cancel=stop)
