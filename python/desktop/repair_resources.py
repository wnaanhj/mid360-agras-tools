# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 估算修复任务的内存开销和可用容量；开工前按请求规模检查，避免对大点云盲目分配整份副本。
"""Physical RAM estimates for repair kernels with optional disk-backed scratch."""
import ctypes
import os
from pathlib import Path


_MIB = 1024**2
_GIB = 1024**3
_NAMES = {
    'radius': '半径去噪', 'statistical': '统计去噪', 'components': '碎片清理',
    'ground': '识别地面', 'level': '整体调平', 'flatten': '局部平整',
    'planes': '平面修复', 'fill': '孔洞补点', 'dynamic': '动态候选',
}


# 提供可用阶段内存信息；从磁盘读取文本。
def available_memory():
    """Report available physical RAM without requiring a third-party dependency."""
    if os.name == 'nt':
        # 定义内存状态的接口。
        class MemoryStatus(ctypes.Structure):
            _fields_ = [('length', ctypes.c_uint32), ('load', ctypes.c_uint32)] + [
                (name, ctypes.c_uint64) for name in (
                    'total_phys', 'avail_phys', 'total_page', 'avail_page',
                    'total_virtual', 'avail_virtual', 'avail_extended')]
        status = MemoryStatus()
        status.length = ctypes.sizeof(status)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return int(status.avail_phys)
    else:
        try:
            for line in Path('/proc/meminfo').read_text().splitlines():
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) * 1024
        except (OSError, ValueError):
            pass
        try:
            pages, size = os.sysconf('SC_AVPHYS_PAGES'), os.sysconf('SC_PAGE_SIZE')
            if pages > 0 and size > 0:
                return int(pages * size)
        except (AttributeError, OSError, ValueError):
            pass
    # Conservative fallback for systems without an available-memory counter.
    return 512 * _MIB


# 提供 memory_plan 信息；输入为 operation、count、selected_count，返回包含 estimated_bytes、budget_bytes 等字段的字典。
def memory_plan(operation, count, selected_count=None, budget=None):
    """Estimate additional resident workspace, excluding disk-backed arrays."""
    selected = int(count if selected_count is None else selected_count)
    budget = int(available_memory() * .45) if budget is None else int(budget)
    batch = min(selected, 8192)
    rates = {
        'radius': (80, 48), 'statistical': (80, 2304), 'components': (352, 16384),
        'ground': (0, 400), 'level': (0, 400), 'flatten': (80, 4096),
        'planes': (0, 400), 'fill': (0, 448), 'dynamic': (0, 128),
    }
    persistent, per_batch = rates[operation]
    estimate = selected * persistent + batch * per_batch
    if operation in ('ground', 'level', 'planes', 'fill'):
        # Covariance fit sample and RANSAC residual matrix are both bounded.
        estimate += max(min(selected, 100_000) * 64, min(selected, 4096) * 480)
    return {'estimated_bytes': estimate, 'budget_bytes': budget}


# 要求内存；输入为 operation、estimated_bytes、budget_bytes，非法输入或处理失败时抛出异常，调用 ValueError 处理。
def require_memory(operation, estimated_bytes, budget_bytes):
    if estimated_bytes > budget_bytes:
        raise ValueError(f'「{_NAMES[operation]}」预计需 {estimated_bytes / _GIB:.4f} GB 内存，'
                         f'当前可用预算 {budget_bytes / _GIB:.4f} GB，内存不足。')


# 检查内存；输入为 operation、count、selected_count，返回 plan。
def check_memory(operation, count, selected_count=None, budget=None):
    plan = memory_plan(operation, count, selected_count, budget)
    require_memory(operation, **plan)
    return plan
