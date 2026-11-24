# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 中文源码导读｜验证与基准
# 职责：UDP点云/IMU格式、单位、原始点序及接收缓冲。
# 此文件用于构造场景、运行断言或生成评估记录，不是日常软件启动入口。
# 运行范围由本文件参数、mock及主入口决定；手动验证可能创建输出、编译内核或打开自建窗口。
"""T100点云结构和本地UDP回归，不给雷达发包。"""
import sys
import socket
import struct
import time
import unittest
from unittest.mock import Mock,patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'radar_client'))
import numpy as np
from t100_stream import decode_packet,StreamState,Receiver


# 测试辅助入口 packet：服务于“UDP点云/IMU格式、单位、原始点序及接收缓冲”场景。
# 输入参数：kind、payload、count；具体构造与使用见下方数据流。
# 返回路径：bytes(data)。
def packet(kind,payload,count=1):
    data=bytearray(36)+payload
    struct.pack_into('<BHHHHBBB',data,0,0,len(data),100,count,12,3,kind,0)
    struct.pack_into('<Q',data,28,12345678)
    return bytes(data)


# 验证分组 T100Tests：UDP点云/IMU格式、单位、原始点序及接收缓冲。
# 本组组织的用例/夹具入口：test_coordinate_units_and_tags、test_spherical_axis_and_zero_returns、test_reject_malformed、test_capacity_expiry_and_recovery、test_mapping_callback_reuses_single_decode_and_rejects_bad_packet、test_loopback_listener_stops_cleanly、test_burst_receiver_preserves_all_packets_and_original_point_slots。
class T100Tests(unittest.TestCase):
    # 回归场景 test_coordinate_units_and_tags：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 数值验收：p.points 与 [[1, -2, 3]] 应在指定容差内一致。
    # 验收：int(p.tags[0]) 应与 16 相等。
    # 验收：int(p.reflectivity[0]) 应与 80 相等。
    def test_coordinate_units_and_tags(self):
        p=decode_packet(packet(1,struct.pack('<iiiBB',1000,-2000,3000,80,16)))
        # 数值验收：p.points 与 [[1, -2, 3]] 应在指定容差内一致。
        np.testing.assert_allclose(p.points,[[1,-2,3]])
        # 验收：int(p.tags[0]) 应与 16 相等。
        # 验收：int(p.reflectivity[0]) 应与 80 相等。
        self.assertEqual(int(p.tags[0]),16);self.assertEqual(int(p.reflectivity[0]),80)
        # 验收：p.timestamp 应与 12345678 相等。
        self.assertEqual(p.timestamp,12345678)
        q=decode_packet(packet(2,struct.pack('<hhhBB',100,-200,300,1,2)))
        # 数值验收：q.points 与 [[1, -2, 3]] 应在指定容差内一致。
        np.testing.assert_allclose(q.points,[[1,-2,3]])

    # 回归场景 test_spherical_axis_and_zero_returns：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 数值验收：q.points 与 [[0, 2, 0]] 应在指定容差内一致（atol=1e-05）。
    # 验收：len(q.points) 应与 0 相等。
    # 验收：len(decode_packet(packet(0, bytes(24))).points) 应与 0 相等。
    def test_spherical_axis_and_zero_returns(self):
        q=decode_packet(packet(3,struct.pack('<IHHBB',2000,9000,9000,1,0)))
        # 数值验收：q.points 与 [[0, 2, 0]] 应在指定容差内一致（atol=1e-05）。
        np.testing.assert_allclose(q.points,[[0,2,0]],atol=1e-5)
        q=decode_packet(packet(1,struct.pack('<iiiBB',0,0,0,0,0)))
        # 验收：len(q.points) 应与 0 相等。
        self.assertEqual(len(q.points),0)
        # 验收：len(decode_packet(packet(0, bytes(24))).points) 应与 0 相等。
        self.assertEqual(len(decode_packet(packet(0,bytes(24))).points),0)

    # 回归场景 test_reject_malformed：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 异常边界：该路径预期抛出 ValueError。
    def test_reject_malformed(self):
        good=packet(1,struct.pack('<iiiBB',1,2,3,0,0))
        cases=[b'bad',good[:-1],packet(9,b''),packet(1,b'',1),packet(3,struct.pack('<IHHBB',1,20000,0,0,0))]
        for raw in cases:
            # 异常边界：该路径预期抛出 ValueError。
            with self.assertRaises(ValueError):decode_packet(raw)

    # 回归场景 test_capacity_expiry_and_recovery：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 验收：len(xyz) 应与 2 相等。
    # 验收：m['evicted'] 应与 3 相等。
    # 验收：len(s.snapshot()[0]) 应与 0 相等。
    def test_capacity_expiry_and_recovery(self):
        s=StreamState(max_points=2);raw=packet(1,struct.pack('<iiiBB',1000,0,0,1,0))
        now=time.monotonic()
        for i in range(5):s.ingest(raw,'127.0.0.1',now-.5+i*.1)
        # 验收：len(xyz) 应与 2 相等。
        # 验收：m['evicted'] 应与 3 相等。
        xyz,_,_,m=s.snapshot(1);self.assertEqual(len(xyz),2);self.assertEqual(m['evicted'],3)
        # 验收：len(s.snapshot()[0]) 应与 0 相等。
        s.last_cloud_at=now-3;self.assertEqual(len(s.snapshot()[0]),0)
        # 验收条件应为假：s.ingest(b'bad', '127.0.0.1')。
        # 验收：s.invalid 应与 1 相等。
        self.assertFalse(s.ingest(b'bad','127.0.0.1'));self.assertEqual(s.invalid,1)
        # 验收：s.error 应与 '' 相等。
        s.ingest(raw,'127.0.0.1');self.assertEqual(s.error,'')

    # 回归场景 test_mapping_callback_reuses_single_decode_and_rejects_bad_packet：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 验收：callback.call_args.args[1] 必须为同一对象 s.last_packet。
    # 验收条件应为假：s.ingest(b'bad', '127.0.0.1', decoded_callback=callback)。
    # 验收条件应为真：s.ingest(raw, '127.0.0.1', decoded_callback=callback)。
    def test_mapping_callback_reuses_single_decode_and_rejects_bad_packet(self):
        s=StreamState();callback=Mock();raw=packet(1,struct.pack('<iiiBB',1000,0,0,1,0))
        with patch('t100_stream.decode_packet',wraps=decode_packet) as decoder:
            # 验收条件应为真：s.ingest(raw, '127.0.0.1', decoded_callback=callback)。
            self.assertTrue(s.ingest(raw,'127.0.0.1',decoded_callback=callback))
            decoder.assert_called_once_with(raw)
        # 验收：callback.call_args.args[1] 必须为同一对象 s.last_packet。
        callback.assert_called_once();self.assertIs(callback.call_args.args[1],s.last_packet)
        # 验收条件应为假：s.ingest(b'bad', '127.0.0.1', decoded_callback=callback)。
        self.assertFalse(s.ingest(b'bad','127.0.0.1',decoded_callback=callback));callback.assert_called_once()

    # 回归场景 test_loopback_listener_stops_cleanly：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 验收条件应为假：r.thread.is_alive()。
    # 验收：s.error 应与 '' 相等。
    # 验收：s.packets 应与 1 相等。
    def test_loopback_listener_stops_cleanly(self):
        s=StreamState();r=Receiver(s,'127.0.0.1',0,'127.0.0.1');r.start()
        sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            sender.sendto(packet(1,struct.pack('<iiiBB',1000,2000,3000,1,0)),('127.0.0.1',r.port))
            end=time.monotonic()+2
            while s.packets<1 and time.monotonic()<end:time.sleep(.01)
            # 验收：s.packets 应与 1 相等。
            self.assertEqual(s.packets,1)
        finally:sender.close();r.close()
        # 验收条件应为假：r.thread.is_alive()。
        self.assertFalse(r.thread.is_alive())
        # 验收：s.error 应与 '' 相等。
        self.assertEqual(s.error,'')

    # 回归场景 test_burst_receiver_preserves_all_packets_and_original_point_slots：围绕“UDP点云/IMU格式、单位、原始点序及接收缓冲”检查下面列出的输入/状态与结果约束。
    # 验收：s.packets 应与 48 相等。
    # 验收：len(received) 应与 48 相等。
    # 验收：p.point_indices 应与 [1] 相等。
    def test_burst_receiver_preserves_all_packets_and_original_point_slots(self):
        s=StreamState();r=Receiver(s,'127.0.0.1',0,'127.0.0.1');r.start()
        sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);received=[]
        r.decoded_callback=lambda raw,p:received.append(p)
        raw=packet(1,struct.pack('<iiiBB',0,0,0,0,0)+struct.pack('<iiiBB',1500,2000,-500,1,2),2)
        try:
            for _ in range(48):sender.sendto(raw,('127.0.0.1',r.port))
            deadline=time.monotonic()+3
            while s.packets<48 and time.monotonic()<deadline:time.sleep(.01)
            # 验收：s.packets 应与 48 相等。
            # 验收：len(received) 应与 48 相等。
            self.assertEqual(s.packets,48);self.assertEqual(len(received),48)
            for p in received:
                # 验收：p.point_indices 应与 [1] 相等。
                np.testing.assert_array_equal(p.point_indices,[1])
                # 数值验收：p.points 与 [[1.5, 2, -0.5]] 应在指定容差内一致。
                np.testing.assert_allclose(p.points,[[1.5,2,-.5]])
        finally:sender.close();r.close()


if __name__=='__main__':unittest.main()
