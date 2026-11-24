// SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
// 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
package cn.t100.mapping;

import java.io.IOException;
import java.util.Arrays;

/** Ethernet II, ARP and unfragmented IPv4/UDP on the ECM link. */
/** 源码维护说明：以用户态应用内部约定解析和构造IPv4、ARP及UDP帧；不持有设备或线程状态。 */
final class UsbFrames {
    /** 静态帧工具不需要实例。 */
    private UsbFrames() {}

    /** 把四段十进制IPv4编码为网络顺序整型，拒绝非法地址。 */
    static int ip(String s) throws IOException {
        if (s == null) throw new IOException("OTG 需要指定 IPv4 地址");
        String[] parts = s.trim().split("\\.", -1);
        int v = 0;
        if (parts.length != 4) throw new IOException("无效的 OTG IPv4 地址：" + s);
        for (String part : parts) {
            if (!part.matches("[0-9]{1,3}")) throw new IOException("无效的 OTG IPv4 地址：" + s);
            int n = Integer.parseInt(part);
            if (n > 255) throw new IOException("无效的 OTG IPv4 地址：" + s);
            v = (v << 8) | n;
        }
        int first = v >>> 24;
        if (first == 0 || first == 127 || first >= 224) throw new IOException("OTG 需要单播 IPv4 地址：" + s);
        return v;
    }
    /** 从字节数组偏移读取网络序16位无符号字段。 */
    static int u16(byte[] b,int p) { return ((b[p]&255)<<8)|(b[p+1]&255); }
    /** 从指定偏移读取网络序32位字段。 */
    static int i32(byte[] b,int p) { return (u16(b,p)<<16)|u16(b,p+2); }
    /** 向目标偏移写网络序16位值。 */
    static void w16(byte[] b,int p,int v) { b[p]=(byte)(v>>>8); b[p+1]=(byte)v; }
    /** 以两个16位写入组合网络序32位值。 */
    static void w32(byte[] b,int p,int v) { w16(b,p,v>>>16); w16(b,p+2,v); }
    /** 比较两个偏移处的完整六字节MAC。 */
    static boolean sameMac(byte[] a,int p,byte[] b,int q) {
        for (int i=0;i<6;i++) if (a[p+i]!=b[q+i]) return false;
        return true;
    }
    /** 检查MAC是否为非零单播地址。 */
    static boolean unicast(byte[] b,int p) { return (b[p]&1)==0 && !all(b,p,(byte)0); }
    /** 判断完整六字节地址是否都是给定值。 */
    private static boolean all(byte[] b,int p,byte value) {
        for (int i=0;i<6;i++) if (b[p+i]!=value) return false;
        return true;
    }
    /** 验证完整以太网及ARP头、合法来源和操作，非有效ARP返回零。 */
    static int arpOperation(byte[] b,int n,byte[] own) {
        if (n<42 || u16(b,12)!=0x806 || u16(b,14)!=1 || u16(b,16)!=0x800
                || b[18]!=6 || b[19]!=4 || !unicast(b,22) || !sameMac(b,6,b,22)
                || sameMac(b,22,own,0)) return 0;
        int op=u16(b,20);
        if(op==1 && (sameMac(b,0,own,0) || all(b,0,(byte)255))) return 1;
        if(op==2 && ((sameMac(b,0,own,0) && sameMac(b,32,own,0)) || all(b,0,(byte)255))) return 2;
        return 0;
    }
    /** 匹配本机目标IP与MAC；ARP请求和应答各用相应字段规则。 */
    static boolean arpForUs(byte[] b,int op,int local,byte[] own) {
        if(i32(b,38)!=local) return false;
        return op==1 ? all(b,32,(byte)0)||sameMac(b,32,own,0)
                : op==2 && sameMac(b,0,own,0) && sameMac(b,32,own,0);
    }
    /** 按请求或应答构造填充到最小长度的以太网ARP帧。 */
    static byte[] arp(byte[] own,int source,int target,byte[] peer) {
        byte[] b=new byte[60];
        if(peer==null) Arrays.fill(b,0,6,(byte)255);
        else System.arraycopy(peer,0,b,0,6);
        System.arraycopy(own,0,b,6,6);
        w16(b,12,0x806); w16(b,14,1); w16(b,16,0x800);
        b[18]=6; b[19]=4; w16(b,20,peer==null?1:2);
        System.arraycopy(own,0,b,22,6);
        w32(b,28,source);
        if(peer!=null) System.arraycopy(peer,0,b,32,6);
        w32(b,38,target);
        return b;
    }
    /** 逐层验证以太网、可选VLAN、IP长度与UDP校验，返回UDP头偏移或失败。 */
    static int udpOffset(byte[] b,int n,int radar,int local,byte[] own) {
        if(n<42 || !sameMac(b,0,own,0) || !unicast(b,6) || sameMac(b,6,own,0)) return -1;
        int h=14,type=u16(b,12);
        if(type==0x8100 || type==0x88a8) {
            if(n<46) return -1;
            h=18; type=u16(b,16);
        }
        if(type!=0x800 || (b[h]&240)!=64 || b[h+9]!=17) return -1;
        int ihl=(b[h]&15)*4,len=u16(b,h+2);
        if(ihl<20 || len<ihl+8 || len>n-h || (u16(b,h+6)&0xbfff)!=0
                || i32(b,h+12)!=radar || i32(b,h+16)!=local
                || fold(sum(b,h,ihl,0))!=0xffff) return -1;
        int u=h+ihl,ul=u16(b,u+4);
        if(ul!=len-ihl || u16(b,u+2)==0) return -1;
        if(u16(b,u+6)!=0 && fold(sum(b,u,ul,pseudo(radar,local,ul)))!=0xffff) return -1;
        return u;
    }
    /** 构造含校验的IPv4 UDP帧，USB末尾填充不改变IP长度。 */
    static byte[] udp(byte[] own,byte[] peer,int local,int radar,int sourcePort,
                      int destinationPort,byte[] payload,int maxPacket) {
        int ul=8+payload.length;
        int length=Math.max(60,34+ul);
        // End the ECM transfer with a short USB packet without changing the IP length.
        if(length%maxPacket==0) length++;
        byte[] b=new byte[length];
        System.arraycopy(peer,0,b,0,6);
        System.arraycopy(own,0,b,6,6);
        w16(b,12,0x800); b[14]=0x45; w16(b,16,20+ul);
        w16(b,20,0x4000); b[22]=64; b[23]=17;
        w32(b,26,local); w32(b,30,radar);
        w16(b,24,fold(sum(b,14,20,0))^0xffff);
        w16(b,34,sourcePort); w16(b,36,destinationPort); w16(b,38,ul);
        System.arraycopy(payload,0,b,42,payload.length);
        int checksum=fold(sum(b,34,ul,pseudo(local,radar,ul)))^0xffff;
        w16(b,40,checksum==0?0xffff:checksum);
        return b;
    }
    /** 按UDP校验规则构造IPv4伪头加和。 */
    private static int pseudo(int source,int target,int ul) {
        return (source>>>16)+(source&0xffff)+(target>>>16)+(target&0xffff)+17+ul;
    }
    /** 按16位字累加指定字节区间，奇数末字节放高位。 */
    private static int sum(byte[] b,int p,int n,int acc) {
        int end=p+n;
        for(;p+1<end;p+=2) acc+=u16(b,p);
        return p==end?acc:acc+((b[p]&255)<<8);
    }
    /** 反复折叠进位到16位，供Internet校验使用。 */
    private static int fold(int n) {
        while((n>>>16)!=0) n=(n&0xffff)+(n>>>16);
        return n;
    }
}
