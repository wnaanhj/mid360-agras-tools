// SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
// 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
package cn.t100.mapping;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.CancellationException;

/** White surface from the exposed faces of occupied voxels; holes are not invented. */
/** 源码维护说明：将占用体素外表面写成OBJ或GLB；输出是近似白模，入口支持取消和同目录临时文件。 */
public final class WhiteMesh {
    /** 静态白模导出工具无需实例。 */
    private WhiteMesh() {}
    private static final int LIMIT=1<<20;
    private static final int[][] N={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
    // Counterclockwise winding, viewed from the outside.
    private static final int[][] F={{0,4,6,2},{1,3,7,5},{0,1,5,4},{2,6,7,3},{0,2,3,1},{4,5,7,6}};
    private static final int[] TRI={0,1,2,0,2,3};
    /** 建立稀疏占用网格并统计外露面的顶点预算。 */
    private static final class Grid {
        final double d,ox,oy,oz;
        final HashSet<Long> occupied=new HashSet<>();
        int vertices;
        int minX=Integer.MAX_VALUE,minY=Integer.MAX_VALUE,minZ=Integer.MAX_VALUE;
        int maxX=Integer.MIN_VALUE,maxY=Integer.MIN_VALUE,maxZ=Integer.MIN_VALUE;
        /** 检查坐标与尺寸，以首点为原点建立体素并计数外壳。 */
        Grid(float[] a,double d,PointTools.Progress p) throws IOException {
            if(a==null||a.length==0||a.length%3!=0||!Double.isFinite(d)||d<=0)
                throw new IOException("需要非空点云和正的体素边长");
            this.d=d;ox=a[0];oy=a[1];oz=a[2];
            for(int i=0;i<a.length;i+=3){
                if(i%3072==0)tick(p,(int)(28L*i/a.length),"收集占用体素");
                int x=index(a[i],ox,d),y=index(a[i+1],oy,d),z=index(a[i+2],oz,d);
                long k=key(x,y,z);
                occupied.add(k);
                minX=Math.min(x,minX);minY=Math.min(y,minY);minZ=Math.min(z,minZ);
                maxX=Math.max(x+1,maxX);maxY=Math.max(y+1,maxY);maxZ=Math.max(z+1,maxZ);
            }
            int i=0;
            for(long k:occupied){
                if((i&1023)==0)tick(p,28+(int)(17L*i/occupied.size()),"统计外表面");i++;
                int x=x(k),y=y(k),z=z(k);
                for(int[] n:N)if(!occupied.contains(key(x+n[0],y+n[1],z+n[2]))){
                    if(vertices>45_000_000-6)throw new IOException("白模规模过大，请增大体素边长");
                    vertices+=6;
                }
            }
        }
        /** 把米坐标换算到有界相对格，拒绝非有限值和过大跨度。 */
        private static int index(float v,double o,double d) throws IOException {
            double f=Math.floor((v-o)/d);
            if(!Double.isFinite(f)||f < -LIMIT||f>=LIMIT)throw new IOException("点云坐标无效或跨度过大，请检查坐标或增大体素边长");
            return (int)f;
        }
    }
    /** 将三轴格打包为长整型，越界用哨兵。 */
    private static long key(int x,int y,int z){
        if(x < -LIMIT || x>=LIMIT || y < -LIMIT || y>=LIMIT || z < -LIMIT || z>=LIMIT)return Long.MIN_VALUE;
        return ((long)(x+LIMIT)<<42)|((long)(y+LIMIT)<<21)|(z+LIMIT);
    }
    /** 从打包键还原第一轴。 */
    private static int x(long k){return (int)(k>>>42)-LIMIT;}
    /** 从打包键还原第二轴。 */
    private static int y(long k){return (int)((k>>>21)&0x1fffff)-LIMIT;}
    /** 从打包键还原第三轴。 */
    private static int z(long k){return (int)(k&0x1fffff)-LIMIT;}
    /** 根据线程或回调停止生成。 */
    private static void tick(PointTools.Progress p,int percent,String status) {
        if(Thread.currentThread().isInterrupted()||(p!=null&&!p.update(percent,status)))throw new CancellationException();
    }
    /** 不带进度生成OBJ白模。 */
    public static String exportObj(File file,float[] xyz,double voxel) throws IOException {return exportObj(file,xyz,voxel,null);}
    /** 在同目录临时文件写入占用格外露三角面，成功后替换目标。 */
    public static String exportObj(File file,float[] xyz,double voxel,PointTools.Progress p) throws IOException {
        Grid g=new Grid(xyz,voxel,p);
        File tmp=temp(file);
        try {
            try(BufferedWriter w=new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tmp),StandardCharsets.UTF_8),64*1024)){
                w.write("# T100 occupied-voxel surface; meters, original point-cloud axes\no T100_white_model\n");
                int i=0,base=1;
                for(long k:g.occupied){
                    if((i&1023)==0)tick(p,45+(int)(54L*i/g.occupied.size()),"写入 OBJ 表面");i++;
                    int x=x(k),y=y(k),z=z(k);
                    for(int face=0;face<6;face++){
                        int[] n=N[face];
                        if(g.occupied.contains(key(x+n[0],y+n[1],z+n[2])))continue;
                        for(int c:F[face]){
                            double px=g.ox+(x+(c&1))*g.d,py=g.oy+(y+((c>>>1)&1))*g.d,pz=g.oz+(z+((c>>>2)&1))*g.d;
                            w.write("v "+px+" "+py+" "+pz+"\n");
                        }
                        w.write("f "+base+" "+(base+1)+" "+(base+2)+"\nf "+base+" "+(base+2)+" "+(base+3)+"\n");
                        base+=4;
                    }
                }
            }
            tick(p,100,"完成 OBJ 白模");replace(tmp,file);
        }finally{if(tmp.exists())tmp.delete();}
        return report(g);
    }
    /** 不带进度生成GLB白模。 */
    public static String exportGlb(File file,float[] xyz,double voxel) throws IOException {return exportGlb(file,xyz,voxel,null);}
    /** 写二进制glTF头和顶点；以X、Z、负Y转换到Y向上坐标系。 */
    public static String exportGlb(File file,float[] xyz,double voxel,PointTools.Progress p) throws IOException {
        Grid g=new Grid(xyz,voxel,p);
        int bin=g.vertices*12;
        // glTF uses Y up, so map source XYZ to X,Z,-Y without reversing winding.
        String min=bounds(g.ox+g.minX*g.d,g.oz+g.minZ*g.d,-(g.oy+g.maxY*g.d));
        String max=bounds(g.ox+g.maxX*g.d,g.oz+g.maxZ*g.d,-(g.oy+g.minY*g.d));
        String json="{\"asset\":{\"version\":\"2.0\",\"generator\":\"T100 voxel surface\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0,\"NORMAL\":1},\"material\":0}]}],\"materials\":[{\"pbrMetallicRoughness\":{\"baseColorFactor\":[1,1,1,1],\"metallicFactor\":0,\"roughnessFactor\":1}}],\"buffers\":[{\"byteLength\":"+(bin*2)+"}],\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":"+bin+",\"target\":34962},{\"buffer\":0,\"byteOffset\":"+bin+",\"byteLength\":"+bin+",\"target\":34962}],\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":"+g.vertices+",\"type\":\"VEC3\",\"min\":"+min+",\"max\":"+max+"},{\"bufferView\":1,\"componentType\":5126,\"count\":"+g.vertices+",\"type\":\"VEC3\"}]}";
        byte[] j=json.getBytes(StandardCharsets.UTF_8);
        int padded=(j.length+3)&~3;
        File tmp=temp(file);
        try {
            try(OutputStream out=new BufferedOutputStream(new FileOutputStream(tmp),64*1024)){
                integer(out,0x46546c67);integer(out,2);integer(out,28+padded+bin*2);
                integer(out,padded);integer(out,0x4e4f534a);out.write(j);
                for(int i=j.length;i<padded;i++)out.write(32);
                integer(out,bin*2);integer(out,0x004e4942);
                for(int pass=0;pass<2;pass++){
                    int i=0;
                    for(long k:g.occupied){
                        if((i&1023)==0)tick(p,45+pass*27+(int)(27L*i/g.occupied.size()),pass==0?"写入 GLB 表面":"写入 GLB 法线");i++;
                        int x=x(k),y=y(k),z=z(k);
                        for(int face=0;face<6;face++){
                            int[] n=N[face];
                            if(g.occupied.contains(key(x+n[0],y+n[1],z+n[2])))continue;
                            for(int t:TRI){
                                if(pass==1){flt(out,n[0]);flt(out,n[2]);flt(out,-n[1]);continue;}
                                int c=F[face][t];
                                flt(out,g.ox+(x+(c&1))*g.d);
                                flt(out,g.oz+(z+((c>>>2)&1))*g.d);
                                flt(out,-(g.oy+(y+((c>>>1)&1))*g.d));
                            }
                        }
                    }
                }
            }
            tick(p,100,"完成 GLB 白模");replace(tmp,file);
        }finally{if(tmp.exists())tmp.delete();}
        return report(g);
    }
    /** 把三个坐标格式化为GLB边界数组文本。 */
    private static String bounds(double a,double b,double c) {
        return "["+(float)a+","+(float)b+","+(float)c+"]";
    }
    /** 说明占用体素、面数及体素近似厚度。 */
    private static String report(Grid g){
        return "体素外表面白模："+g.occupied.size()+" 个占用体素，"+g.vertices/3+" 个三角面；薄片厚度约为体素边长";
    }
    /** 把浮点数按小端IEEE754写入二进制载荷。 */
    private static void flt(OutputStream out,double f) throws IOException {integer(out,Float.floatToIntBits((float)f));}
    /** 按最低字节在前写四字节整数。 */
    private static void integer(OutputStream out,int n) throws IOException {out.write(n);out.write(n>>>8);out.write(n>>>16);out.write(n>>>24);}
    /** 在目标同级目录建立独立临时输出文件。 */
    private static File temp(File f) throws IOException {return File.createTempFile("mesh_",".tmp",f.getAbsoluteFile().getParentFile());}
    /** 将已经完整写出的临时文件移动到调用方指定目标。 */
    private static void replace(File src,File dst) throws IOException {
        java.nio.file.Files.move(src.toPath(),dst.toPath(),java.nio.file.StandardCopyOption.REPLACE_EXISTING);
    }
}
