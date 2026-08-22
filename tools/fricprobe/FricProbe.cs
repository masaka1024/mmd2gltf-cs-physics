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

        int frames = EnvI("FRAMES", 120);
        var start = new Vec3[world.Bodies.Count];
        for (int i = 0; i < world.Bodies.Count; i++) start[i] = world.Bodies[i].WorldTransform.Origin;
        for (int f = 0; f < frames; f++) world.StepSimulation(1f / 60f);

        var sb = new StringBuilder();
        sb.Append("  [実効] FricMul=").Append(world.FrictionCombineMultiply)
          .Append("  FricAligned=").Append(world.FrictionVelocityAligned)
          .Append("  PoolOrder=").Append(world.ContactPoolOrder)
          .Append("  NormalFirst=").Append(world.ContactNormalBeforeFriction)
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
