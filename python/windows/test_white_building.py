# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 中文源码导读｜验证与基准
# 职责：房屋墙面、主方向和有实测支持的几何建模。
# 此文件用于构造场景、运行断言或生成评估记录，不是日常软件启动入口。
# 运行范围由本文件参数、mock及主入口决定；手动验证可能创建输出、编译内核或打开自建窗口。
"""Planar reconstruction: geometric evidence, gap preservation, cancellation."""
from pathlib import Path
import sys
import threading
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from desktop.white_building import build_building


# 测试辅助入口 _sheet：服务于“房屋墙面、主方向和有实测支持的几何建模”场景。
# 输入参数：axis、offset、a、b、step；具体构造与使用见下方数据流。
# 返回路径：p。
def _sheet(axis,offset,a,b,step=.036):
    cols=[x for x in range(3) if x!=axis]
    u,v=np.meshgrid(np.arange(*a,step),np.arange(*b,step))
    p=np.zeros((u.size,3))
    p[:,axis]=offset
    p[:,cols]=np.column_stack([u.ravel(),v.ravel()])
    return p


# 验证分组 BuildingTests：房屋墙面、主方向和有实测支持的几何建模。
# 本组组织的用例/夹具入口：test_rotated_room_retains_door_and_small_hole_can_close、test_separated_coplanar_patches_and_small_object、test_adjacent_wall_and_floor_share_mesh_vertices、test_sparse_cloud_and_cancellation。
class BuildingTests(unittest.TestCase):
    # 回归场景 test_rotated_room_retains_door_and_small_hole_can_close：围绕“房屋墙面、主方向和有实测支持的几何建模”检查下面列出的输入/状态与结果约束。
    # 验收：d['wall_surfaces'] 应与 4 相等。
    # 验收：d['horizontal_surfaces'] 应与 2 相等。
    # 验收条件应为假：np.any((c[:, 0] > 1.09) & (c[:, 0] < 1.88) & (c[:, 2] > 0.2) & (c[:, 2] < 1.87))。
    def test_rotated_room_retains_door_and_small_hole_can_close(self):
        front=_sheet(1,0,(0,5),(0,2.6))
        door=(front[:,0]>.95)&(front[:,0]<2.02)&(front[:,2]<2.04)
        pinhole=(front[:,0]>3.28)&(front[:,0]<3.39)&(front[:,2]>1.30)&(front[:,2]<1.43)
        room=np.concatenate([front[~(door|pinhole)],_sheet(1,4,(0,5),(0,2.6)),
             _sheet(0,0,(0,4),(0,2.6)),_sheet(0,5,(0,4),(0,2.6)),
             _sheet(2,0,(0,5),(0,4)),_sheet(2,2.6,(0,5),(0,4))])
        rng=np.random.default_rng(2026)
        room+=rng.normal(0,.005,room.shape)
        a=np.deg2rad(21);c=np.cos(a);s=np.sin(a)
        R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        v,f,d=build_building(room@R.T + [2,8,1],.08)
        # 验收：d['wall_surfaces'] 应与 4 相等。
        self.assertEqual(d['wall_surfaces'],4,d)
        # 验收：d['horizontal_surfaces'] 应与 2 相等。
        self.assertEqual(d['horizontal_surfaces'],2,d)
        p=(v-[2,8,1])@R
        tris=p[f]
        wall=np.abs(tris[:,:,1]).max(axis=1)<.05
        c=tris[wall].mean(axis=1)
        # 验收条件应为假：np.any((c[:, 0] > 1.09) & (c[:, 0] < 1.88) & (c[:, 2] > 0.2) & (c[:, 2] < 1.87))。
        self.assertFalse(np.any((c[:,0]>1.09)&(c[:,0]<1.88)&(c[:,2]>.2)&(c[:,2]<1.87)))
        # 验收：np.min(np.linalg.norm(c[:, [0, 2]] - [3.34, 1.35], axis=1)) 必须小于 0.075。
        self.assertLess(np.min(np.linalg.norm(c[:,[0,2]]-[3.34,1.35],axis=1)),.075)
        # 验收：(p[:, 1] < 0.05).sum() 必须大于 500。
        self.assertGreater((p[:,1]<.05).sum(),500)
        # 验收条件应为真：np.all(np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1) > 1e-08)。
        self.assertTrue(np.all(np.linalg.norm(np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]),axis=1)>1e-8))

    # 回归场景 test_separated_coplanar_patches_and_small_object：围绕“房屋墙面、主方向和有实测支持的几何建模”检查下面列出的输入/状态与结果约束。
    # 验收：d['wall_surfaces'] 应与 2 相等。
    # 验收：d['horizontal_surfaces'] 应与 0 相等。
    # 验收条件应为假：np.any((centers[:, 0] > 2.15) & (centers[:, 0] < 3.45))。
    def test_separated_coplanar_patches_and_small_object(self):
        cloud=np.concatenate([_sheet(1,0,(0,2),(0,2.4)),_sheet(1,0,(3.6,5.9),(0,2.4)),
                             _sheet(1,0,(8,8.3),(0,.3)),_sheet(2,.82,(1.,1.6),(1.5,2.1))])
        v,f,d=build_building(cloud,.10)
        # 验收：d['wall_surfaces'] 应与 2 相等。
        self.assertEqual(d['wall_surfaces'],2,d)
        # 验收：d['horizontal_surfaces'] 应与 0 相等。
        self.assertEqual(d['horizontal_surfaces'],0,d)
        centers=v[f].mean(axis=1)
        # 验收条件应为假：np.any((centers[:, 0] > 2.15) & (centers[:, 0] < 3.45))。
        self.assertFalse(np.any((centers[:,0]>2.15)&(centers[:,0]<3.45)))
        # 验收：v[:, 0].max() 必须小于 6.05。
        self.assertLess(v[:,0].max(),6.05)

    # 回归场景 test_adjacent_wall_and_floor_share_mesh_vertices：围绕“房屋墙面、主方向和有实测支持的几何建模”检查下面列出的输入/状态与结果约束。
    # 验收：len(np.intersect1d(f[axes == first], f[axes == second])) 必须大于 8。
    def test_adjacent_wall_and_floor_share_mesh_vertices(self):
        p=np.concatenate([_sheet(0,0,(0,2),(0,2.3)),_sheet(1,0,(0,3),(0,2.3)),
            _sheet(2,0,(0,3),(0,2)),_sheet(0,3,(0,2),(0,2.3)),
            _sheet(1,2,(0,3),(0,2.3))])
        v,f,_=build_building(p,.08)
        normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        axes=np.argmax(np.abs(normals),axis=1)
        for first,second in ((0,1),(0,2),(1,2)):
            # 验收：len(np.intersect1d(f[axes == first], f[axes == second])) 必须大于 8。
            self.assertGreater(len(np.intersect1d(f[axes==first],f[axes==second])),8)

    # 回归场景 test_sparse_cloud_and_cancellation：围绕“房屋墙面、主方向和有实测支持的几何建模”检查下面列出的输入/状态与结果约束。
    # 验收：(v.shape, f.shape) 应与 ((0, 3), (0, 3)) 相等。
    # 异常边界：该路径预期抛出 InterruptedError。
    # 异常边界：该路径预期抛出 ValueError。
    def test_sparse_cloud_and_cancellation(self):
        cloud=np.arange(36.).reshape(12,3)
        v,f,d=build_building(cloud,.08)
        # 验收：(v.shape, f.shape) 应与 ((0, 3), (0, 3)) 相等。
        self.assertEqual((v.shape,f.shape),((0,3),(0,3)))
        event=threading.Event()
        # 异常边界：该路径预期抛出 InterruptedError。
        with self.assertRaises(InterruptedError):
            build_building(cloud,.08,event, lambda msg:event.set())
        # 异常边界：该路径预期抛出 ValueError。
        with self.assertRaises(ValueError):
            build_building(cloud,np.nan)
