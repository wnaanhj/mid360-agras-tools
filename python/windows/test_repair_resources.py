# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 中文源码导读｜验证与基准
# 职责：修复资源预算、可用内存和索引开销估算。
# 此文件用于构造场景、运行断言或生成评估记录，不是日常软件启动入口。
# 运行范围由本文件参数、mock及主入口决定；手动验证可能创建输出、编译内核或打开自建窗口。
"""Regression coverage for disk-backed repair and resource estimates."""
from contextlib import contextmanager
from pathlib import Path
import tempfile
import unittest

import numpy as np

from desktop.repair_core import RepairCancelled, apply_operation
from desktop.repair_resources import available_memory, check_memory, memory_plan
from windows.test_repair_core import grid


# 测试辅助入口 disk_arrays：服务于“修复资源预算、可用内存和索引开销估算”场景。
# 返回路径：arr；np.empty(shape, dtype)。
@contextmanager
def disk_arrays():
    with tempfile.TemporaryDirectory() as folder:
        mappings = []

        # 测试辅助入口 allocate：服务于“修复资源预算、可用内存和索引开销估算”场景。
        # 输入参数：shape、dtype、fill；具体构造与使用见下方数据流。
        # 返回路径：arr；np.empty(shape, dtype)。
        def allocate(shape, dtype, fill=None):
            if not np.prod(shape):
                return np.empty(shape, dtype)
            path = Path(folder) / f'{len(mappings)}.bin'
            arr = np.memmap(path, mode='w+', shape=shape, dtype=dtype)
            mappings.append(arr)
            if fill is not None:
                arr.fill(fill)
            return arr

        try:
            yield allocate
        finally:
            for arr in mappings:
                arr._mmap.close()


# 验证分组 RepairResourcesTests：修复资源预算、可用内存和索引开销估算。
# 本组组织的用例/夹具入口：test_all_operations_match_ram_and_disk、test_budget_uses_actual_selection_before_allocating、test_large_empty_selection_and_disk_cancellation。
class RepairResourcesTests(unittest.TestCase):
    # 回归场景 test_all_operations_match_ram_and_disk：围绕“修复资源预算、可用内存和索引开销估算”检查下面列出的输入/状态与结果约束。
    # 验收：set(disk.stats) 应与 set(ram.stats) 相等。
    # 验收：getattr(disk, attr) 应与 getattr(ram, attr) 相等。
    # 验收：disk.stats[key] 应与 ram.stats[key] 相等。
    def test_all_operations_match_ram_and_disk(self):
        q = grid(38, 38)
        keep = np.ones(len(q), bool)
        keep[5 * 38 + 4] = False  # measured floor has a real gap
        q = np.vstack((q[keep], grid(21, 21) + [0, 0, 1.2], [[4, 4, 5]])).astype(np.float32)
        selection = np.arange(len(q)) < len(q) - 1
        for operation in ('radius', 'statistical', 'components', 'ground', 'level', 'flatten', 'planes', 'fill'):
            with self.subTest(operation=operation), disk_arrays() as allocator:
                ram = apply_operation(q, operation, selection=selection)
                disk = apply_operation(q, operation, selection=selection, allocator=allocator)
                for attr in ('points', 'keep', 'moved', 'added'):
                    # 验收：getattr(disk, attr) 应与 getattr(ram, attr) 相等。
                    np.testing.assert_array_equal(getattr(disk, attr), getattr(ram, attr))
                self.assertIsInstance(disk.points, np.memmap)
                # 验收：set(disk.stats) 应与 set(ram.stats) 相等。
                self.assertEqual(set(disk.stats), set(ram.stats))
                for key in ram.stats:
                    # 验收：disk.stats[key] 应与 ram.stats[key] 相等。
                    np.testing.assert_array_equal(disk.stats[key], ram.stats[key])
                if operation == 'fill':
                    # 验收：len(disk.added) 必须大于 0。
                    self.assertGreater(len(disk.added), 0)
                elif operation == 'planes':
                    # 验收：disk.stats['planes_found'] 必须大于等于 2。
                    self.assertGreaterEqual(disk.stats['planes_found'], 2)
                elif operation == 'ground':
                    self.assertIsInstance(disk.stats['ground_mask'], np.memmap)
                    # 验收条件应为假：disk.stats['ground_mask'][-1]。
                    self.assertFalse(disk.stats['ground_mask'][-1])

    # 回归场景 test_budget_uses_actual_selection_before_allocating：围绕“修复资源预算、可用内存和索引开销估算”检查下面列出的输入/状态与结果约束。
    # 验收：available_memory() 必须大于 0。
    # 验收：check_memory('radius', 16000000, 20, budget) 应与 small 相等。
    # 验收：small['estimated_bytes'] 必须小于 small['budget_bytes']。
    def test_budget_uses_actual_selection_before_allocating(self):
        # 验收：available_memory() 必须大于 0。
        self.assertGreater(available_memory(), 0)
        budget = 100_000_000
        small = memory_plan('radius', 16_000_000, 20, budget)
        # 验收：check_memory('radius', 16000000, 20, budget) 应与 small 相等。
        self.assertEqual(check_memory('radius', 16_000_000, 20, budget), small)
        # 验收：small['estimated_bytes'] 必须小于 small['budget_bytes']。
        self.assertLess(small['estimated_bytes'], small['budget_bytes'])
        # 异常边界：该路径预期抛出 ValueError，同时匹配 '半径去噪.*GB.*可用预算.*GB'。
        with self.assertRaisesRegex(ValueError, '半径去噪.*GB.*可用预算.*GB'):
            check_memory('radius', 16_000_000, budget=budget)
        q = grid(51, 51)
        before = q.copy()

        # 测试辅助入口 unexpected：服务于“修复资源预算、可用内存和索引开销估算”场景。
        # 主要协作调用：self.fail；此入口通过断言/状态变更完成检查。
        def unexpected(*args):
            self.fail('预算不足时不应创建结果或工作文件')

        # 异常边界：该路径预期抛出 ValueError，同时匹配 '碎片清理.*内存不足'。
        with self.assertRaisesRegex(ValueError, '碎片清理.*内存不足'):
            apply_operation(q, 'components', allocator=unexpected, memory_budget=1000)
        # 验收：q 应与 before 相等。
        np.testing.assert_array_equal(q, before)

    # 回归场景 test_large_empty_selection_and_disk_cancellation：围绕“修复资源预算、可用内存和索引开销估算”检查下面列出的输入/状态与结果约束。
    # 验收：last[0] 必须小于 1.0。
    # 验收：result.stats['input_points'] 应与 n 相等。
    # 验收：result.stats['selected_points'] 应与 0 相等。
    def test_large_empty_selection_and_disk_cancellation(self):
        n = 5_000_001
        q = np.broadcast_to(np.array([[10, 20, 30]], dtype=np.float32), (n, 3))
        selection = np.broadcast_to(False, (n,))
        with disk_arrays() as allocator:
            result = apply_operation(q, 'radius', selection=selection, allocator=allocator, memory_budget=0)
            self.assertIsInstance(result.points, np.memmap)
            # 验收：result.stats['input_points'] 应与 n 相等。
            self.assertEqual(result.stats['input_points'], n)
            # 验收：result.stats['selected_points'] 应与 0 相等。
            self.assertEqual(result.stats['selected_points'], 0)
            # 验收：result.stats['removed_points'] 应与 0 相等。
            self.assertEqual(result.stats['removed_points'], 0)
            # 验收：result.points[-1] 应与 q[-1] 相等。
            np.testing.assert_array_equal(result.points[-1], q[-1])
        last = [0.]
        # 异常边界：该路径预期抛出 RepairCancelled。
        with disk_arrays() as allocator, self.assertRaises(RepairCancelled):
            apply_operation(grid(101, 101), 'radius', allocator=allocator,
                            cancel=lambda: last[0] > 0,
                            progress=lambda value: last.__setitem__(0, value))
        # 验收：last[0] 必须小于 1.0。
        self.assertLess(last[0], 1.)


if __name__ == '__main__':
    unittest.main()
