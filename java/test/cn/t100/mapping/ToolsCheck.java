// SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
// 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
/* 中文源码导读｜验证与实验：点云修复工具的选区、应用、撤销与几何结果。 * 此目录不是正式应用启动入口；运行main/仪器测试可能产生测试文件或访问指定USB设备。 * 合成与离线测试验证指定边界，不构成真实雷达精度或长期吞吐保证。 */
package cn.t100.mapping;

import java.io.File;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.CancellationException;

/* 验证/实验类型 ToolsCheck：点云修复工具的选区、应用、撤销与几何结果。 */
public final class ToolsCheck {
    /* 辅助步骤 assertTrue：配合本文件的“点云修复工具的选区、应用、撤销与几何结果”场景完成数据准备、处理或核对。 输入：boolean b,String msg。 */
    static void assertTrue(boolean b,String msg){/* 失败检查分支：(!b)；进入后按下面的断言路径报告不满足的条件。 */
    if(!b)throw new AssertionError(msg);}
    /* 辅助步骤 patch：配合本文件的“点云修复工具的选区、应用、撤销与几何结果”场景完成数据准备、处理或核对。 输入：int n,double step,double tilt,boolean hole。 返回：out。 */
    static float[] patch(int n,double step,double tilt,boolean hole){
        ArrayList<Float> a=new ArrayList<>();
        for(int i=0;i<n;i++)for(int j=0;j<n;j++){
            if(hole&&i==n/2&&j==n/2)continue;
            a.add((float)(i*step));a.add((float)(j*step));a.add((float)(i*step*tilt));
        }
        float[] out=new float[a.size()];for(int i=0;i<out.length;i++)out[i]=a.get(i);return out;
    }
    /* 辅助步骤 append：配合本文件的“点云修复工具的选区、应用、撤销与几何结果”场景完成数据准备、处理或核对。 输入：float[] a,float... b。 返回：c。 */
    static float[] append(float[] a,float... b){float[] c=Arrays.copyOf(a,a.length+b.length);System.arraycopy(b,0,c,a.length,b.length);return c;}
    /* 辅助步骤 apply：配合本文件的“点云修复工具的选区、应用、撤销与几何结果”场景完成数据准备、处理或核对。 输入：String op,float[] pts,double d,boolean[] keep。 返回：true | r。 */
    static PointTools.Result apply(String op,float[] pts,double d,boolean[] keep){
        int[] prev={0};
        PointTools.Result r=PointTools.apply(op,pts,d,keep,(pc,msg)->{
            assertTrue(pc>=prev[0]&&pc<=100,"progress "+op+" "+pc+" after "+prev[0]);prev[0]=pc;
            return true;
        });
        assertTrue(prev[0]==100,"finish "+op);return r;
    }
    /* 辅助步骤 distance：配合本文件的“点云修复工具的选区、应用、撤销与几何结果”场景完成数据准备、处理或核对。 输入：float[] a,int i,int j。 返回：(float)Math.sqrt(x * x + y * y + z * z)。 */
    static float distance(float[] a,int i,int j){
        double x=a[i*3]-a[j*3],y=a[i*3+1]-a[j*3+1],z=a[i*3+2]-a[j*3+2];
        return (float)Math.sqrt(x*x+y*y+z*z);
    }
    /* 解析本验证入口参数，按函数体顺序执行场景并输出结果。 输入：String[] args。 返回：true。 */
    public static void main(String[] args) throws Exception {
        float[] dense=patch(20,.04,0,false), noisy=append(dense,8,8,8,8.01f,8,8,10,10,10);
        assertTrue(apply("radius",noisy,.1,null).points.length==dense.length,"radius");
        assertTrue(apply("statistical",noisy,3,null).points.length==dense.length,"statistical");
        assertTrue(apply("fragments",noisy,.16,null).points.length==dense.length,"fragments");
        float[] floor=patch(40,.09,.28,false), slope=append(floor,1,1,3,2,2,3,1,2,3);
        assertTrue(apply("ground",slope,.12,null).points.length==floor.length,"ground");
        float[] level=apply("level",slope,.12,null).points;
        assertTrue(Math.abs(distance(slope,0,slope.length/3-1)-distance(level,0,level.length/3-1))<1e-5,"level rigid");
        assertTrue(Math.abs(level[2]-level[floor.length-1])<.03,"level horizontal");
        boolean[] select=new boolean[slope.length/3];Arrays.fill(select,0,floor.length/3,true);
        float[] uneven=slope.clone();uneven[3*980+2]+=.06f;
        float[] flat=apply("flatten",uneven,.12,select).points;
        assertTrue(Math.abs(uneven[3*980+2]-flat[3*980+2])>.02,"flatten changes local height");
        assertTrue(flat[flat.length-1]==uneven[uneven.length-1],"flatten protects points elsewhere");
        float[] projected=apply("plane",uneven,0,select).points;
        assertTrue(Math.abs(projected[3*980+2]-projected[3*980]*.28)<.002,"projection");
        assertTrue(projected[projected.length-1]==uneven[uneven.length-1],"projection outside selection");
        assertTrue(apply("crop",slope,0,select).points.length==floor.length,"crop");
        assertTrue(apply("delete",slope,0,select).points.length==9,"delete");
        float[] hole=patch(7,.25,0,true);
        PointTools.Result filled=apply("fill",hole,.25,null);
        assertTrue(filled.points.length==hole.length+3&&filled.inferred[hole.length/3],"hole fill inferred");
        assertTrue(Arrays.equals(hole,Arrays.copyOf(filled.points,hole.length)),"fill preserves input");
        boolean[] markerSelection=new boolean[filled.points.length/3];
        markerSelection[0]=markerSelection[markerSelection.length-1]=true;
        PointTools.Result cropped=PointTools.apply("crop",filled.points,0,markerSelection,filled.inferred,null);
        assertTrue(Arrays.equals(cropped.inferred,new boolean[]{false,true}),"crop preserves inferred provenance");
        boolean[] holes=new boolean[hole.length/3];Arrays.fill(holes,true);
        assertTrue(apply("fill",hole,.25,holes).points.length==hole.length+3,"hole fill selected");
        try{PointTools.apply("radius",dense,.1,null,(pc,msg)->false);throw new AssertionError("cancel");}
        catch(CancellationException expected){}
        try{PointTools.apply("temporal",dense,0,null,null);throw new AssertionError("unjustified temporal");}
        catch(IllegalArgumentException expected){assertTrue(expected.getMessage().contains("逐帧"),"temporal explanation");}
        float[] candidates={0,0,0,3,3,0,9,9,0};
        PointTools.TimedFrame[] views={
            new PointTools.TimedFrame(0,new float[]{0,0,0,3,3,0}),
            new PointTools.TimedFrame(2,new float[]{.3f,0,0,3,3,0}),
            new PointTools.TimedFrame(4,new float[]{.3f,0,0,3,3,0})
        };
        int[] temporalProgress={0};
        PointTools.TemporalResult temporal=PointTools.dynamicCandidates(candidates,null,views,.10,.5,1,(pc,msg)->{
            assertTrue(pc>=temporalProgress[0],"dynamic progress");temporalProgress[0]=pc;return true;
        });
        assertTrue(temporalProgress[0]==100&&Arrays.equals(temporal.candidates,new boolean[]{true,false,false}),"candidate evidence");
        Path directory=Files.createTempDirectory("t100_tools_test");
        File obj=directory.resolve("mesh.obj").toFile(),glb=directory.resolve("mesh.glb").toFile();
        try {
            float[] pair={0,0,0,1.2f,0,0};
            String objReport=WhiteMesh.exportObj(obj,pair,1);
            assertTrue(objReport.contains("20 个三角面"),"OBJ report");
            assertTrue(Files.readAllLines(obj.toPath()).stream().filter(s->s.startsWith("f ")).count()==20,"interior face removed");
            byte[] original=Files.readAllBytes(obj.toPath());
            try{WhiteMesh.exportObj(obj,pair,1,(pc,msg)->pc<45);throw new AssertionError("cancel not supported");}
            catch(CancellationException expected){}
            assertTrue(Arrays.equals(original,Files.readAllBytes(obj.toPath())),"cancel preserves existing output");
            WhiteMesh.exportGlb(glb,pair,1);
            ByteBuffer b=ByteBuffer.wrap(Files.readAllBytes(glb.toPath())).order(ByteOrder.LITTLE_ENDIAN);
            assertTrue(b.getInt()==0x46546c67&&b.getInt()==2&&b.getInt()==b.capacity(),"GLB header");
            int json=b.getInt();assertTrue(b.getInt()==0x4e4f534a,"GLB JSON");
            b.position(b.position()+json);
            int bin=b.getInt();assertTrue(bin==60*24&&b.getInt()==0x004e4942&&b.remaining()==bin,"GLB triangles and normals");
            int offset=b.position();
            for(int i=0;i<60;i+=3){
                double[] u=new double[3],v=new double[3];
                for(int axis=0;axis<3;axis++){
                    float origin=b.getFloat(offset+i*12+axis*4);
                    u[axis]=b.getFloat(offset+(i+1)*12+axis*4)-origin;
                    v[axis]=b.getFloat(offset+(i+2)*12+axis*4)-origin;
                }
                int normal=offset+60*12+i*12;
                double dot=(u[1]*v[2]-u[2]*v[1])*b.getFloat(normal)+(u[2]*v[0]-u[0]*v[2])*b.getFloat(normal+4)+(u[0]*v[1]-u[1]*v[0])*b.getFloat(normal+8);
                assertTrue(dot>0,"winding");
            }
            ToolHistory history=new ToolHistory(directory.toFile());
            history.record(hole,null);history.record(filled.points,filled.inferred);
            assertTrue(Arrays.equals(history.undo().points,hole),"history undo");
            assertTrue(Arrays.equals(history.redo().inferred,filled.inferred),"history redo");
            history.undo();history.record(dense,null);
            assertTrue(!history.canRedo()&&Arrays.equals(history.undo().points,hole),"redo branch");
            history.reset();assertTrue(!history.canUndo(),"history reset");
        }finally{
            try(java.util.stream.Stream<Path> list=Files.list(directory)){for(Path p:(Iterable<Path>)list::iterator)Files.delete(p);}
            Files.delete(directory);
        }
        System.out.println("TOOLS_CHECK_OK");
    }
}


