// タスク2: swing-twist 分解 (TwistAngleDeg) の単体テスト。
// 既知入力(純スイング/純ツイスト/合成, 90°超・180°付近を含む)で解析期待値と照合。
using System;
using BulletPhysics;

namespace BoneCheck
{
    public static class SwingTwistTest
    {
        static int _fail = 0;
        static Quat AxisAngle(Vec3 axis, float deg) =>
            Quat.FromAxisAngle(axis, deg * (float)Math.PI / 180f);

        static void Chk(string name, float got, float exp, float tol)
        {
            // 角度は ±360 の周期。最短差で比較。
            float d = got - exp;
            while (d > 180) d -= 360; while (d < -180) d += 360;
            bool ok = Math.Abs(d) <= tol;
            Console.WriteLine($"  [{(ok ? "PASS" : "FAIL")}] {name}: got={got:F2}° exp={exp:F2}° (差{d:F2})");
            if (!ok) _fail++;
        }

        public static bool Run()
        {
            _fail = 0;
            Console.WriteLine("== swing-twist 分解 単体テスト (twist軸=+Y) ==");
            var Y = Vec3.YAxis;

            // 1) 純ツイスト(Yまわり) → twist = その角
            foreach (float th in new[] { 10f, 30f, 90f, 150f, -45f, -120f })
                Chk($"純ツイストY {th}°", SkirtMeasure.TwistAngleDeg(AxisAngle(Y, th), Y), th, 0.05f);

            // 2) 純スイング(X/Zまわり, 90°超含む) → twist = 0
            foreach (var ax in new[] { Vec3.XAxis, Vec3.ZAxis })
                foreach (float ph in new[] { 10f, 30f, 90f, 120f, 150f, 170f })
                    Chk($"純スイング{(ax.x != 0 ? "X" : "Z")} {ph}°", SkirtMeasure.TwistAngleDeg(AxisAngle(ax, ph), Y), 0f, 0.05f);

            // 3) 合成 q = swing(X,φ) * twist(Y,θ)。分解で twist=θ を復元できるか (90°超含む)。
            foreach (float ph in new[] { 20f, 60f, 90f, 120f, 150f })
                foreach (float th in new[] { 5f, 30f, -40f, 80f })
                {
                    var q = AxisAngle(Vec3.XAxis, ph) * AxisAngle(Y, th); // swing*twist
                    Chk($"合成 swingX{ph}*twistY{th}", SkirtMeasure.TwistAngleDeg(q, Y), th, 0.3f);
                }

            // 4) 特異点: swing≈180° (Xまわり180°) + 小ツイスト。twist未定義域の挙動を明示。
            {
                var q = AxisAngle(Vec3.XAxis, 179.5f) * AxisAngle(Y, 5f);
                float got = SkirtMeasure.TwistAngleDeg(q, Y);
                Console.WriteLine($"  [INFO] 特異点 swingX179.5*twistY5: got={got:F2}° (180°付近は twist不安定。仕様: w²+射影²<1e-6 で0を返す)");
            }
            {
                var q = AxisAngle(Vec3.XAxis, 180f); // 純180°swing, twistなし
                float got = SkirtMeasure.TwistAngleDeg(q, Y);
                Chk("特異点 純swingX180° → 0", got, 0f, 0.5f);
            }

            Console.WriteLine(_fail == 0 ? "== swing-twist: 全PASS ==" : $"== swing-twist: {_fail}件FAIL ==");
            return _fail == 0;
        }
    }
}
