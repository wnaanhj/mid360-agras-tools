# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 中文源码导读｜验证与基准
# 职责：离群点、碎片、平面、地面及补洞的几何运算。
# 此文件用于构造场景、运行断言或生成评估记录，不是日常软件启动入口。
# 运行范围由本文件参数、mock及主入口决定；手动验证可能创建输出、编译内核或打开自建窗口。
"""Regression tests for independent geometric point-cloud repairs."""
import threading
import unittest

import numpy as np

from desktop.repair_core import RepairCancelled, apply_operation


# 测试辅助入口 grid：服务于“离群点、碎片、平面、地面及补洞的几何运算”场景。
# 输入参数：cols、rows、step；具体构造与使用见下方数据流。
# 返回路径：np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))).astype(np.float32)。
def grid(cols=21, rows=21, step=.05):
    x, y = np.meshgrid(np.arange(cols) * step, np.arange(rows) * step)
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))).astype(np.float32)


# 验证分组 RepairCoreTests：离群点、碎片、平面、地面及补洞的几何运算。
# 本组组织的用例/夹具入口：test_radius_inclusive_and_protects_unselected、test_statistical_rejects_remote_point、test_components_checks_real_distance_and_extent、test_ground_prefers_low_plane_without_changing_other_heights、test_ground_uses_low_spatial_evidence_under_dense_roof、test_ground_and_level_decline_high_planes_over_uncertain_lower_returns、test_level_uses_reference_but_rigidly_transforms_entire_cloud、test_flatten_keeps_vertical_wall_step_and_unselected_point、test_planes_fits_wall_and_respects_selection_and_limit、test_fill_adds_small_interior_gap_only_with_budget 等。
class RepairCoreTests(unittest.TestCase):
    # 回归场景 test_radius_inclusive_and_protects_unselected：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收：result.keep 应与 [True, True, False, True] 相等。
    # 验收：q 应与 original 相等。
    # 验收：result.points 应与 original 相等。
    def test_radius_inclusive_and_protects_unselected(self):
        q = np.array([[0, 0, 0], [.125, 0, 0], [6, 0, 0], [7, 0, 0]], np.float32)
        original = q.copy()
        result = apply_operation(q, 'radius', {'radius': .125, 'min_neighbors': 1},
                                 np.array([True, True, True, False]))
        # 验收：result.keep 应与 [True, True, False, True] 相等。
        np.testing.assert_array_equal(result.keep, [True, True, False, True])
        # 验收：q 应与 original 相等。
        np.testing.assert_array_equal(q, original)
        # 验收：result.points 应与 original 相等。
        np.testing.assert_array_equal(result.points, original)
        # 验收条件应为假：result.moved.any()。
        self.assertFalse(result.moved.any())
        # 验收：apply_operation(q, 'radius', selection=np.zeros(4, bool)).keep 应与 True 相等。
        np.testing.assert_array_equal(apply_operation(q, 'radius', selection=np.zeros(4, bool)).keep, True)

    # 回归场景 test_statistical_rejects_remote_point：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收：result.keep[:-1] 应与 True 相等。
    # 验收条件应为假：result.keep[-1]。
    def test_statistical_rejects_remote_point(self):
        q = np.vstack([grid(7, 7), np.array([[10, 10, 8]], np.float32)])
        result = apply_operation(q, 'statistical', {'neighbors': 6})
        # 验收：result.keep[:-1] 应与 True 相等。
        np.testing.assert_array_equal(result.keep[:-1], True)
        # 验收条件应为假：result.keep[-1]。
        self.assertFalse(result.keep[-1])

    # 回归场景 test_components_checks_real_distance_and_extent：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收：result.keep[:36] 应与 True 相等。
    # 验收：result.keep[36:40] 应与 False 相等。
    # 验收：result.keep[40:] 应与 [True, False, False] 相等。
    def test_components_checks_real_distance_and_extent(self):
        q = np.vstack([grid(6, 6), grid(2, 2) + np.float32(2),
                       [[-0.075, 0, 0], [-0.16, .06, 0]], [[8., 8., 8.]]]).astype(np.float32)
        result = apply_operation(q, 'components', {'distance': .081, 'min_points': 4, 'min_extent': .1})
        # 验收：result.keep[:36] 应与 True 相等。
        np.testing.assert_array_equal(result.keep[:36], True)
        # 验收：result.keep[36:40] 应与 False 相等。
        np.testing.assert_array_equal(result.keep[36:40], False)  # four points, but small extent
        # 验收：result.keep[40:] 应与 [True, False, False] 相等。
        np.testing.assert_array_equal(result.keep[40:], [True, False, False])
        dense = np.vstack([np.repeat(q[:1], 1024, axis=0),
                           np.repeat(q[1:2], 1024, axis=0), q[-1:]])
        result = apply_operation(dense, 'components', {'distance': .081, 'min_points': 4, 'min_extent': 0.})
        # 验收：result.keep[:-1] 应与 True 相等。
        np.testing.assert_array_equal(result.keep[:-1], True)
        # 验收条件应为假：result.keep[-1]。
        self.assertFalse(result.keep[-1])

    # 回归场景 test_ground_prefers_low_plane_without_changing_other_heights：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收：clean.stats['ground_mask'][:len(low)] 应与 True 相等。
    # 验收：clean.stats['ground_mask'][len(low):] 应与 False 相等。
    # 验收：clean.points 应与 q 相等。
    def test_ground_prefers_low_plane_without_changing_other_heights(self):
        low = grid(18, 18)
        roof = grid(18, 18) + np.array([0., 0., .60], dtype=np.float32)
        wall = grid(18, 18)[:, [2, 0, 1]] + np.array([1.4, 0., .1], dtype=np.float32)
        q = np.vstack((low, roof, wall))
        clean = apply_operation(q, 'ground')
        # 验收：clean.stats['ground_mask'][:len(low)] 应与 True 相等。
        np.testing.assert_array_equal(clean.stats['ground_mask'][:len(low)], True)
        # 验收：clean.stats['ground_mask'][len(low):] 应与 False 相等。
        np.testing.assert_array_equal(clean.stats['ground_mask'][len(low):], False)
        # 验收：clean.points 应与 q 相等。
        np.testing.assert_array_equal(clean.points, q)
        # 验收条件应为假：clean.moved.any()。
        self.assertFalse(clean.moved.any())
        slope = low.copy()
        slope[:, 2] = slope[:, 0] * 2
        # 验收条件应为假：apply_operation(slope, 'ground').stats['ground_mask'].any()。
        self.assertFalse(apply_operation(slope, 'ground').stats['ground_mask'].any())

    # 回归场景 test_ground_uses_low_spatial_evidence_under_dense_roof：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收条件应为真：repair.stats['ground_found']。
    # 验收：repair.stats['ground_mask'][:len(ground)] 应与 True 相等。
    # 验收条件应为假：repair.stats['ground_mask'][len(ground):].any()。
    def test_ground_uses_low_spatial_evidence_under_dense_roof(self):
        rng = np.random.default_rng(721)
        ground = grid(9, 9, .25)
        ground[:, 2] = .16 * ground[:, 0] - .12 * ground[:, 1]
        roof = grid(42, 42, 2. / 41)
        roof[:, 2] = 2.75 + rng.uniform(-.003, .003, len(roof))
        cabinet = grid(18, 18, .05) + np.array([.55, .55, 1.4], np.float32)
        stray = np.column_stack((rng.uniform(0, 2, size=(8, 2)), rng.uniform(-1.2, -.6, size=8)))
        q = np.vstack((ground, roof, cabinet, stray)).astype(np.float32)
        q += np.array([12., -4., 55.], np.float32)
        repair = apply_operation(q, 'ground')
        # 验收条件应为真：repair.stats['ground_found']。
        self.assertTrue(repair.stats['ground_found'])
        # 验收：repair.stats['ground_mask'][:len(ground)] 应与 True 相等。
        np.testing.assert_array_equal(repair.stats['ground_mask'][:len(ground)], True)
        # 验收条件应为假：repair.stats['ground_mask'][len(ground):].any()。
        self.assertFalse(repair.stats['ground_mask'][len(ground):].any())

    # 回归场景 test_ground_and_level_decline_high_planes_over_uncertain_lower_returns：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收条件应为假：result.stats['ground_found']。
    # 验收条件应为假：result.stats['ground_mask'].any()。
    # 验收：result.points 应与 q 相等。
    def test_ground_and_level_decline_high_planes_over_uncertain_lower_returns(self):
        rng = np.random.default_rng(32)
        roof = grid(41, 41, .05) + np.array([0., 0., 3.5], np.float32)
        cabinet = grid(18, 18, .05) + np.array([.55, .55, 2.4], np.float32)
        uncertain = grid(17, 17, .125)
        uncertain[:, 2] = rng.uniform(-2.5, .5, len(uncertain))
        q = np.vstack((uncertain, roof, cabinet))
        result = apply_operation(q, 'ground')
        # 验收条件应为假：result.stats['ground_found']。
        self.assertFalse(result.stats['ground_found'])
        # 验收条件应为假：result.stats['ground_mask'].any()。
        self.assertFalse(result.stats['ground_mask'].any())
        result = apply_operation(q, 'level')
        # 验收条件应为假：result.stats['ground_found']。
        self.assertFalse(result.stats['ground_found'])
        # 验收：result.points 应与 q 相等。
        np.testing.assert_array_equal(result.points, q)
        # 验收：result.stats['transform'] 应与 np.eye(4) 相等。
        np.testing.assert_array_equal(result.stats['transform'], np.eye(4))
    # 回归场景 test_level_uses_reference_but_rigidly_transforms_entire_cloud：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 数值验收：result.points[:len(floor), 2] 与 -0.25 应在指定容差内一致（atol=2e-06）。
    # 验收条件应为真：result.moved[-2:].all()。
    # 数值验收：np.linalg.norm(q[:-1] - q[-1], axis=1) 与 np.linalg.norm(result.points[:-1] - result.points[-1], axis=1) 应在指定容差内一致（atol=2e-06）。
    def test_level_uses_reference_but_rigidly_transforms_entire_cloud(self):
        floor = grid()
        floor[:, 2] = .25 * floor[:, 0] - .15 * floor[:, 1] + .8
        q = np.vstack([floor, np.array([[9, 3, 4], [-4, 2, 5]], np.float32)])
        original = q.copy()
        selection = np.arange(len(q)) < len(floor)
        result = apply_operation(q, 'level', {'target_height': -.25}, selection)
        # 数值验收：result.points[:len(floor), 2] 与 -0.25 应在指定容差内一致（atol=2e-06）。
        np.testing.assert_allclose(result.points[:len(floor), 2], -.25, atol=2e-6)
        # 验收条件应为真：result.moved[-2:].all()。
        self.assertTrue(result.moved[-2:].all())
        # 数值验收：np.linalg.norm(q[:-1] - q[-1], axis=1) 与 np.linalg.norm(result.points[:-1] - result.points[-1], axis=1) 应在指定容差内一致（atol=2e-06）。
        np.testing.assert_allclose(np.linalg.norm(q[:-1] - q[-1], axis=1),
                                   np.linalg.norm(result.points[:-1] - result.points[-1], axis=1), atol=2e-6)
        tf = result.stats['transform']
        # 数值验收：result.points 与 q @ tf[:3, :3].T + tf[:3, 3] 应在指定容差内一致（atol=1e-06）。
        np.testing.assert_allclose(result.points, q @ tf[:3, :3].T + tf[:3, 3], atol=1e-6)
        # 验收：q 应与 original 相等。
        np.testing.assert_array_equal(q, original)
        # 验收：apply_operation(q, 'level', selection=~np.ones(len(q), bool)).points 应与 q 相等。
        np.testing.assert_array_equal(apply_operation(q, 'level', selection=~np.ones(len(q), bool)).points, q)

    # 回归场景 test_flatten_keeps_vertical_wall_step_and_unselected_point：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 数值验收：result.points[index, 2] 与 0.018 应在指定容差内一致（places=6）。
    # 验收：result.points[-len(wall):] 应与 wall 相等。
    # 验收：result.points[untouched] 应与 q[untouched] 相等。
    def test_flatten_keeps_vertical_wall_step_and_unselected_point(self):
        q = grid()
        raised = q[:, 0] > .5
        q[raised, 2] = .18
        index = np.argmin(np.linalg.norm(q[:, :2] - [.25, .25], axis=1))
        q[index, 2] = .028
        untouched = np.argmin(np.linalg.norm(q[:, :2] - [.65, .65], axis=1))
        q[untouched, 2] = .207
        wall = grid(7, 7)[:, [2, 0, 1]] + np.array([1.6, 0, 0], np.float32)
        q = np.vstack([q, wall])
        original = q.copy()
        selection = np.ones(len(q), bool)
        selection[untouched] = False
        result = apply_operation(q, 'flatten', {'max_move': .01}, selection)
        # 数值验收：result.points[index, 2] 与 0.018 应在指定容差内一致（places=6）。
        self.assertAlmostEqual(result.points[index, 2], .018, places=6)
        # 验收：result.points[-len(wall):] 应与 wall 相等。
        np.testing.assert_array_equal(result.points[-len(wall):], wall)
        # 验收：result.points[untouched] 应与 q[untouched] 相等。
        np.testing.assert_array_equal(result.points[untouched], q[untouched])
        # 验收：result.points[:len(raised), 2][raised & selection[:len(raised)]] 应与 original[:len(raised), 2][raised & selection[:len(raised)]] 相等。
        np.testing.assert_array_equal(result.points[:len(raised), 2][raised & selection[:len(raised)]], original[:len(raised), 2][raised & selection[:len(raised)]])
        # 验收：q 应与 original 相等。
        np.testing.assert_array_equal(q, original)

    # 回归场景 test_planes_fits_wall_and_respects_selection_and_limit：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 验收：result.stats['planes_found'] 必须大于 0。
    # 验收：result.points[selection, 0].std() 必须小于 0.55 * q[selection, 0].std()。
    # 验收：np.linalg.norm(result.points - q, axis=1).max() 必须小于等于 0.012001。
    def test_planes_fits_wall_and_respects_selection_and_limit(self):
        q = grid()[:, [2, 0, 1]]
        q[:, 0] = 2. + np.random.default_rng(2).uniform(-.025, .025, len(q))
        original = q.copy()
        selection = np.arange(len(q)) % 7 != 0
        result = apply_operation(q, 'planes', {'max_move': .012, 'strength': 1}, selection)
        # 验收：result.stats['planes_found'] 必须大于 0。
        self.assertGreater(result.stats['planes_found'], 0)
        # 验收：result.points[selection, 0].std() 必须小于 0.55 * q[selection, 0].std()。
        self.assertLess(result.points[selection, 0].std(), .55 * q[selection, 0].std())
        # 验收：np.linalg.norm(result.points - q, axis=1).max() 必须小于等于 0.012001。
        self.assertLessEqual(np.linalg.norm(result.points - q, axis=1).max(), .012001)
        # 验收：result.points[~selection] 应与 q[~selection] 相等。
        np.testing.assert_array_equal(result.points[~selection], q[~selection])
        # 验收：q 应与 original 相等。
        np.testing.assert_array_equal(q, original)

    # 回归场景 test_fill_adds_small_interior_gap_only_with_budget：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 数值验收：result.added[order] 与 expected 应在指定容差内一致（atol=1e-06）。
    # 验收：result.points 应与 src 相等。
    # 验收条件应为假：result.moved.any()。
    def test_fill_adds_small_interior_gap_only_with_budget(self):
        q = grid(38, 38)
        x, y = np.rint(q[:, :2].T / .05).astype(int)
        small = (x >= 4) & (x <= 5) & (y >= 5) & (y <= 6)
        second = (x == 11) & (y == 12)
        large = (x >= 19) & (x <= 33) & (y >= 19) & (y <= 32)
        exterior = (x <= 1) & (y >= 20) & (y <= 25)
        src = q[~(small | second | large | exterior)]
        result = apply_operation(src, 'fill')
        order = np.lexsort((result.added[:, 1], result.added[:, 0]))
        expected = q[small | second]
        expected = expected[np.lexsort((expected[:, 1], expected[:, 0]))]
        # 数值验收：result.added[order] 与 expected 应在指定容差内一致（atol=1e-06）。
        np.testing.assert_allclose(result.added[order], expected, atol=1e-6)
        # 验收：result.points 应与 src 相等。
        np.testing.assert_array_equal(result.points, src)
        # 验收条件应为假：result.moved.any()。
        self.assertFalse(result.moved.any())
        from_selection = apply_operation(q, 'fill', selection=~(small | second | large | exterior))
        # 验收：len(from_selection.added) 应与 0 相等。
        self.assertEqual(len(from_selection.added), 0)
        limited = apply_operation(src, 'fill', {'max_added': 1})
        # 数值验收：limited.added 与 q[second] 应在指定容差内一致（atol=1e-06）。
        np.testing.assert_allclose(limited.added, q[second], atol=1e-6)
        # 验收：apply_operation(src, 'fill', selection=np.zeros(len(src), bool)).added 应与 np.empty((0, 3), dtype=np.float32) 相等。
        np.testing.assert_array_equal(apply_operation(src, 'fill', selection=np.zeros(len(src), bool)).added,
                                      np.empty((0, 3), dtype=np.float32))

    # 回归场景 test_validation_and_cancellation：围绕“离群点、碎片、平面、地面及补洞的几何运算”检查下面列出的输入/状态与结果约束。
    # 异常边界：该路径预期抛出 RepairCancelled，同时匹配 '取消'。
    # 异常边界：该路径预期抛出 RepairCancelled。
    # 异常边界：该路径预期抛出 ValueError，同时匹配 '选区'。
    def test_validation_and_cancellation(self):
        q = grid(101, 101)
        stop = threading.Event()
        # 测试辅助入口 progress：服务于“离群点、碎片、平面、地面及补洞的几何运算”场景。
        # 输入参数：value；具体构造与使用见下方数据流。
        # 主要协作调用：stop.set；此入口通过断言/状态变更完成检查。
        def progress(value):
            if value:
                stop.set()
        # 异常边界：该路径预期抛出 RepairCancelled，同时匹配 '取消'。
        with self.assertRaisesRegex(RepairCancelled, '取消'):
            apply_operation(q, 'radius', cancel=stop, progress=progress)
        # 异常边界：该路径预期抛出 RepairCancelled。
        with self.assertRaises(RepairCancelled):
            apply_operation(q, 'components', cancel=lambda: True)
        # 异常边界：该路径预期抛出 ValueError，同时匹配 '选区'。
        with self.assertRaisesRegex(ValueError, '选区'):
            apply_operation(q, 'radius', selection=np.zeros(len(q)))
        # 异常边界：该路径预期抛出 ValueError，同时匹配 '修复参数'。
        with self.assertRaisesRegex(ValueError, '修复参数'):
            apply_operation(q, 'fill', {'max_added': 100001})
        # 异常边界：该路径预期抛出 ValueError，同时匹配 '有限'。
        with self.assertRaisesRegex(ValueError, '有限'):
            q[0, 0] = np.nan
            apply_operation(q, 'ground')


if __name__ == '__main__':
    unittest.main()
