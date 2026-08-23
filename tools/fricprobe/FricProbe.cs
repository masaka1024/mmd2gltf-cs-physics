// fricprobe: 傾斜面に置いた箱が滑るかどうかだけを見る最小ランナー。
//   摩擦の合成方式 (積 / 幾何平均 / …) を「滑る・止まる」の二値で判別するための装置。
//   PMX をそのまま組んで N フレーム回し、動的剛体の移動量を出す。ネットの刈り込みはしない。
using System;
using System.Globalization;
using System.IO;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;

static class FricProbe
{
    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }

    static int Main()
    {
        string pmx = Env("MMD_TEST_PMX");
        if (string.IsNullOrEmpty(pmx) || !File.Exists(pmx))
        { Console.WriteLine("[SKIP] MMD_TEST_PMX を指定すること"); return 1; }

        // env が明示されたときだけ上書き (未設定 = 出荷既定)。
        if (Env("FRICMUL") != null) { }   // world 生成後に当てる
        var model = PmxReader.LoadFile(pmx);
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        world.SubSteps = EnvI("SUBSTEPS", world.SubSteps);
        world.SolverIterations = EnvI("ITERS", world.SolverIterations);
        if (Env("FRICMUL") != null) world.FrictionCombineMultiply = Env("FRICMUL") == "1";
        if (Env("FRICALIGN") != null) world.FrictionVelocityAligned = Env("FRICALIGN") == "1";
        if (Env("CPOOL") != null) world.ContactPoolOrder = Env("CPOOL") == "1";
        if (Env("NORMFIRST") != null) world.ContactNormalBeforeFriction = Env("NORMFIRST") == "1";
        // ★タスク75: MMD は滑り出した箱を 0.5〜0.7 秒で止める。Bullet の眠り (deactivation) が
        //   効いている可能性を測るための A/B。既定は OFF (出荷既定と同じ)。
        if (Env("SLEEP") != null) world.EnableSleeping = Env("SLEEP") == "1";
        { float v;
          if (float.TryParse(Env("SLEEP_LIN"), NumberStyles.Float, CultureInfo.InvariantCulture, out v)) world.LinearSleepThreshold = v;
          if (float.TryParse(Env("SLEEP_ANG"), NumberStyles.Float, CultureInfo.InvariantCulture, out v)) world.AngularSleepThreshold = v;
          if (float.TryParse(Env("SLEEP_T"), NumberStyles.Float, CultureInfo.InvariantCulture, out v)) world.DeactivationTime = v; }

        int frames = EnvI("FRAMES", 120);
        var start = new Vec3[world.Bodies.Count];
        for (int i = 0; i < world.Bodies.Count; i++) start[i] = world.Bodies[i].WorldTransform.Origin;
        // ★軌跡の書き出し (TRAJ_OUT)。ベイクした VMD の曲線と並べるため。
        //   NetDump はジョイント0本のモデルで箱を「網の外」に落としてしまうので使えない。
        var traj = new StringBuilder();
        string trajOut = Env("TRAJ_OUT");
        if (trajOut != null) traj.Append("frame,name,px,py,pz").Append('\n');
        for (int f = 0; f < frames; f++)
        {
            if (trajOut != null)
                foreach (var bd in world.Bodies)
                    traj.Append(f).Append(',').Append(bd.Name).Append(',')
                        .Append(bd.WorldTransform.Origin.x.ToString("G9", CultureInfo.InvariantCulture)).Append(',')
                        .Append(bd.WorldTransform.Origin.y.ToString("G9", CultureInfo.InvariantCulture)).Append(',')
                        .Append(bd.WorldTransform.Origin.z.ToString("G9", CultureInfo.InvariantCulture)).Append('\n');
            world.StepSimulation(1f / 60f);
        }
        if (trajOut != null) File.WriteAllText(trajOut, traj.ToString(), new UTF8Encoding(false));

        var sb = new StringBuilder();
        sb.Append("  [実効] FricMul=").Append(world.FrictionCombineMultiply)
          .Append("  FricAligned=").Append(world.FrictionVelocityAligned)
          .Append("  PoolOrder=").Append(world.ContactPoolOrder)
          .Append("  NormalFirst=").Append(world.ContactNormalBeforeFriction)
          .Append("  Sleep=").Append(world.EnableSleeping)
          .Append("/").Append(world.LinearSleepThreshold).Append("/").Append(world.DeactivationTime)
          .Append("  SubSteps=").Append(world.SubSteps).Append("  Iters=").Append(world.SolverIterations)
          .Append('\n');
        for (int i = 0; i < world.Bodies.Count; i++)
        {
            var b = world.Bodies[i];
            if (b.IsStaticOrKinematic) continue;
            float d = (b.WorldTransform.Origin - start[i]).Length;
            sb.Append("  ").Append(b.Name.PadRight(10))
              .Append(" 移動 ").Append(d.ToString("F5", CultureInfo.InvariantCulture))
              .Append("   => ").Append(d > 0.5f ? "滑る" : "止まる").Append('\n');
        }
        Console.Write(sb.ToString());
        string outp = Env("OUT");
        if (!string.IsNullOrEmpty(outp)) File.WriteAllText(outp, sb.ToString(), new UTF8Encoding(false));
        return 0;
    }
}
