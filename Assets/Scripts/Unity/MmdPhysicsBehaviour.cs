// ===========================================================================
// Unity Bullet 互換物理エンジン – Unity ブリッジ
// PMX を読み込み、物理ワールドを毎フレーム進め、ボーン Transform と同期する。
// エンジンは PMX ネイティブ座標 (左手系 Y-up) で動作し、境界で Unity 座標へ変換。
// ===========================================================================

using System.Collections.Generic;
using UnityEngine;
using BulletPhysics.Pmx;

namespace BulletPhysics.Unity
{
    /// <summary>MMD/PMX 物理を Unity 上で駆動するコンポーネント。</summary>
    public sealed class MmdPhysicsBehaviour : MonoBehaviour
    {
        public enum InputSource { Pmx, Glb }

        [Tooltip("入力: Pmx=PMX直読み / Glb=GLBのextras.mmd経由 (どちらも同一の物理駆動)")]
        public InputSource Source = InputSource.Pmx;

        [Tooltip("読み込む .pmx ファイルの絶対 or Assets 相対パス (Source=Pmx のとき)")]
        public string PmxPath = "";

        [Tooltip("読み込む .glb ファイルのパス (Source=Glb のとき。extras.mmd から剛体/Joint/ボーンを構築)")]
        public string GlbPath = "";

        [Tooltip("剛体 BoneIndex -> ボーン Transform のマップ (ボーン名で解決)。GLBは import 済みスケルトンのルート")]
        public Transform ModelRoot;

        [Tooltip("エンジン(PMXネイティブ単位) -> Unity 配置スケール。Unity側モデルが縮小配置される運用向け")]
        public float UnitScale = 0.08f;

        [Header("Solver")]
        public float Gravity = 98f;         // MMD スケール重力 (約 9.8 * 10)
        public int SolverIterations = 10;
        // ★既定 = 1/60 × SubSteps 1 (2026-08-09 ジャダー対策で 1/30×2サブ から変更)。
        //   実効刻みは 1/60 で従来と同一 → 本家忠実度は数値まで一致することをヘッドレスで確認済み
        //   (bonecheck 傾き中央11.20 / p90 23.52 / 12窓比1.0611 が変更前と同値)。CPUも同等。
        //   利点: Time.fixedDeltaTime(=1/60, 下の AlignUnityFixedTimestep が自動整列) と一致するため
        //   毎FixedUpdateでちょうど1ステップ進み、更新間隔が等間隔になる(髪/スカートのコマ落ちが消える)。
        //   従来の 1/30 では 1FixedUpdate あたりの内部ステップが 0,1,0,1,1,... と変動し
        //   実時間の更新間隔が 20ms/40ms とバラついていた。詳細は DESIGN.md「コマ落ち(ジャダー)」節。
        //   より忠実にしたい場合は SubSteps を 2 (=実効1/120) に。
        public int SubSteps = 1;
        public float FixedTimeStep = 1f / 60f;

        [Header("Smoothness (コマ落ち/ジャダー対策)")]
        // 症状: 髪やスカートがカクついて見える。原因は「物理の更新間隔が実時間で不均一」なこと。
        //   Unity の FixedUpdate は Time.fixedDeltaTime 間隔(既定0.02s=50Hz)で呼ばれるが、
        //   エンジンは FixedTimeStep(既定1/30) のアキュムレータなので、内部ステップは
        //   実時間 20ms / 40ms とバラバラな間隔でしか進まない (実測: 0,1,0,1,1,... の周期)。
        //   物理は毎回33.3ms分進むのに表示間隔が揃わない=ジャダー。
        // 対策: Time.fixedDeltaTime と FixedTimeStep を一致させ、毎FixedUpdateでちょうど1ステップ進める。
        //   FixedTimeStep=1/60・SubSteps=1 は実効刻みが現行(1/30×2サブ)と同一のため、
        //   ヘッドレス検証で本家忠実度が完全一致することを確認済み (傾き11.20/p90 23.52/12窓比1.0611)。CPUも同等。
        // ★既定ON (2026-08-09)。Unity全体の物理刻み(Time.fixedDeltaTime)を FixedTimeStep に合わせる。
        //   既定 0.02(50Hz) → 1/60(60Hz) になる。Custom運用では PhysX はパーク済みなので実害はない。
        //   他のFixedUpdate処理も60Hzになる点だけ留意 (呼び出し回数が2割増)。OFFにすると未整列時に警告のみ。
        [Tooltip("ONで Time.fixedDeltaTime を FixedTimeStep に合わせる (毎FixedUpdate=1ステップ=等間隔)。Unity全体の物理刻みを変える点に注意")]
        public bool AlignUnityFixedTimestep = true;

        [Header("Startup")]
        // 起動直後、アニメがフレーム0姿勢を確定させた後に物理をボーンへ再整合する遅延(フレーム数)。
        // バインド姿勢→フレーム0への瞬間移動でスカート等が脚へ貫入(突き抜け)するのを防ぐ。
        // Animator は Update と LateUpdate の間で姿勢を書くため、LateUpdate 時点ではフレーム0が
        // 反映されている。そこで FK-rest リセット(ResetBodiesToBonePoseFk 相当)を掛けると、
        // バインド位置に取り残された動的剛体(スカート/髪)が posed 骨格の周りへ置き直される。
        // 0=無効(従来どおり Start 時のバインド基準のみ) / 1=フレーム0適用後の最初のLateUpdate /
        // 2以上=さらに数フレーム保持してからライブ物理へ渡す(取りこぼし保険)。
        [Tooltip("起動直後にアニメのフレーム0姿勢へ物理を再整合する遅延フレーム数。バインド→フレーム0の瞬間移動による貫入(突き抜け)対策。0で無効。")]
        public int PoseResetDelayFrames = 2;

        [Header("Correction (本家PMXエディタの補正層再現)")]
        // [物理+ボーン位置合わせ] 再現: 書き戻し時、位置を「親ボーン(補正済)位置+親回転×bindオフセット」の
        // 階層再構成に置換し、物理の移動分を捨てる (回転は物理のまま)。補正OFF/ON対照データで式を確定済み。
        // 既定 false=従来(物理位置をそのまま書き戻し)。ヘルパは PmxPhysicsBuilder.ComputeAlignedBonePoses (共通)。
        [Tooltip("本家の[ボーン位置合わせ]再現: 位置=親チェーン再構成(移動分を捨てる)/回転=物理。スカート/髪の貫通表示対策。")]
        public bool AlignBonePositions = false;

        // [Jointロック内部演算] 再現の第一形: 親側ジョイントの相対eulerをリミット超過分だけ α で戻す。
        // 0=無効(回転そのまま) / 1=完全clamp。本家ONの超過8-14°は完全clampでないことを示すため中間値。
        // AlignBonePositions とセットで使う (位置だけONは有害と実測済: 深貫入47,749)。掃引結果で既定を更新予定。
        [Tooltip("回転をジョイント角度リミットへ戻す割合 (AlignBonePositionsとセットで使用)。0=無効, 1=完全clamp。")]
        [Range(0f, 1f)] public float AlignRotClampAlpha = 0.5f;


        [Header("Timing Diagnosis (Animator遅れの実測)")]
        // 1フレーム内の実行順序と「物理が見たボーン」vs「表示されるボーン」の遅れを実測する。
        // ONにすると約120描画フレームをログして自動OFF。観点:
        //  - FixedUpdate時のボーン位置(=物理が見る) と LateUpdate時(=Animator適用後, 表示に近い) の差 dFL。
        //    dFL が動きに比例して大きい → 体コライダーは表示より1フレーム古い姿勢 = 速い動きで脚が刺さる機構。
        [Tooltip("ONで約120フレーム、実行順序/dt/ボーン遅れをConsoleへログして自動OFF")]
        public bool DiagnoseTiming = false;
        [Tooltip("遅れ計測に使う速く動くボーン名")]
        public string DiagnoseBone = "右ひざ";

        private int _diagLeft = 0;
        private Transform _diagTr;
        private Vector3 _diagFixedPos, _diagUpdatePos;
        private int _diagFixedCount; private float _diagDt; private int _diagSteps;
        private readonly List<float> _diagDFL = new(); // |Fixed - Late|
        private readonly List<float> _diagDUL = new(); // |Update - Late|
        private static float Dist(Vector3 a, Vector3 b)
        { float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z; return (float)System.Math.Sqrt(dx * dx + dy * dy + dz * dz); }

        void Update()
        {
            if (DiagnoseTiming)
            {
                DiagnoseTiming = false; _diagLeft = 120; _diagDFL.Clear(); _diagDUL.Clear();
                _diagTr = null;
                if (_model != null && _boneTransforms != null)
                    for (int i = 0; i < _model.BoneNames.Count && i < _boneTransforms.Length; i++)
                        if (_model.BoneNames[i] == DiagnoseBone) { _diagTr = _boneTransforms[i]; break; }
                Debug.Log($"[TimingDiag] 開始 bone={DiagnoseBone} 解決={(_diagTr != null)} fixedDeltaTime={Time.fixedDeltaTime:F4} FTS={FixedTimeStep:F4} SubSteps={SubSteps}");
            }
            if (_diagLeft > 0 && _diagTr != null) _diagUpdatePos = _diagTr.position;
        }

        [Header("Debug")]
        public bool DrawGizmos = true;

        private PmxPhysicsBuilder _builder;
        private PmxPhysicsModel _model;
        private Transform[] _boneTransforms;   // BoneIndex -> Transform
        private int _startupResetCountdown = 0; // >0 の間、LateUpdate で posed 姿勢へ再整合する

        void Start()
        {
            if (Source == InputSource.Glb) { if (!string.IsNullOrEmpty(GlbPath)) LoadGlb(GlbPath); }
            else { if (!string.IsNullOrEmpty(PmxPath)) LoadPmx(PmxPath); }
        }

        // PMX 直読み。
        public void LoadPmx(string path) => BuildAndInit(PmxReader.LoadFile(path));

        // GLB の extras.mmd 経由。UnitScale は extras.mmd の値を優先する
        // (GLB のメッシュ/スケルトンはその scale で import されているため、表示境界を一致させる必要がある)。
        public void LoadGlb(string path)
        {
            var model = GlbPhysicsReader.LoadFile(path, out float unitScale, out var warnings);
            if (warnings != null)
                foreach (var w in warnings) Debug.LogWarning($"[MmdPhysics][GLB] {w}");
            if (unitScale > 0f && System.Math.Abs(unitScale - UnitScale) > 1e-6f)
            {
                Debug.Log($"[MmdPhysics][GLB] UnitScale を extras.mmd の値 {unitScale} に設定 (Inspector {UnitScale} を上書き)");
                UnitScale = unitScale;
            }
            BuildAndInit(model);
        }

        // 入力経路に依らない共通の初期化 (物理駆動ロジックの共通化)。起動時に FK-rest リセットを必ず呼ぶ。
        private void BuildAndInit(PmxPhysicsModel model)
        {
            _model = model;
            _builder = PmxPhysicsBuilder.Build(_model);
            _builder.World.Gravity = new Vec3(0f, -Gravity, 0f);
            _builder.World.SolverIterations = SolverIterations;
            _builder.World.SubSteps = SubSteps;
            _builder.World.FixedTimeStep = FixedTimeStep;
            ResolveBones();
            ResetPhysicsToBones();
            // アニメがフレーム0を適用するのは Start より後(Update→LateUpdate 間)。この時点の
            // リセットはバインド基準なので、LateUpdate で posed 姿勢へ再整合し直す予約を入れる。
            _startupResetCountdown = PoseResetDelayFrames > 0 ? PoseResetDelayFrames : 0;
            CheckTimestepAlignment();
        }

        // 物理刻みと Unity の FixedUpdate 間隔が食い違うと、内部ステップが実時間で不均一になり
        // 髪/スカートがカクついて見える (ジャダー)。起動時に1度だけ整列 or 警告する。
        private void CheckTimestepAlignment()
        {
            if (AlignUnityFixedTimestep)
            {
                if (System.Math.Abs(Time.fixedDeltaTime - FixedTimeStep) > 1e-6f)
                {
                    Time.fixedDeltaTime = FixedTimeStep;
                    Debug.Log($"[MmdPhysics] Time.fixedDeltaTime を {FixedTimeStep:F6} に整列しました " +
                              $"(毎FixedUpdateでちょうど1ステップ=等間隔更新。SubSteps={SubSteps} で実効刻み {FixedTimeStep / SubSteps:F6})");
                }
                return;
            }
            float ratio = FixedTimeStep / Time.fixedDeltaTime;
            if (System.Math.Abs(ratio - Mathf_Round(ratio)) > 1e-3f)
                Debug.LogWarning($"[MmdPhysics] 物理刻みが Unity と整列していません: FixedTimeStep={FixedTimeStep:F5} / Time.fixedDeltaTime={Time.fixedDeltaTime:F5} " +
                    $"(比 {ratio:F3})。内部ステップが実時間で不均一(例 20ms/40ms交互)になり、髪やスカートがカクついて見えます。" +
                    "対策: AlignUnityFixedTimestep を ON にするか、FixedTimeStep=1/60・SubSteps=1 にして Fixed Timestep も 0.0166667 に合わせてください。");
        }
        private static float Mathf_Round(float v) => (float)System.Math.Round(v);

        /// <summary>物理開始/リセット時に、全剛体を現在のボーン姿勢へ整合させる
        /// (MMD の物理演算リセット相当)。フレーム0で脚が曲がっていても動的剛体がバインド位置に
        /// 取り残されて貫入するのを防ぐ。LoadPmx 後および任意のタイミングで呼べる。</summary>
        public void ResetPhysicsToBones()
        {
            if (_builder == null) return;
            // FK-rest で統一: 物理ボーン(スカート/髪)はスケルトンの姿勢を使わず親から前計算する。
            // スケルトンの物理ボーンに前フレームの物理結果が残っていても正しい開始状態になる。
            _builder.ResetBodiesToBonePoseFk(BoneWorldOrNull);
        }

        // ボーンindex → ワールド姿勢 (MMD座標)。Transform 未解決なら null (バインド維持)。
        private RigidTransform? BoneWorldOrNull(int boneIndex)
        {
            if (boneIndex < 0 || _boneTransforms == null ||
                boneIndex >= _boneTransforms.Length || _boneTransforms[boneIndex] == null)
                return null;
            var tr = _boneTransforms[boneIndex];
            return new RigidTransform(UnityToMmdRot(tr.rotation), UnityToMmdPos(tr.position));
        }

        private void ResolveBones()
        {
            _boneTransforms = new Transform[_model.BoneNames.Count];
            if (ModelRoot == null) return;
            var map = new Dictionary<string, Transform>();
            foreach (var t in ModelRoot.GetComponentsInChildren<Transform>())
                map[t.name] = t;
            for (int i = 0; i < _model.BoneNames.Count; i++)
                if (map.TryGetValue(_model.BoneNames[i], out var tr))
                    _boneTransforms[i] = tr;
        }

        void FixedUpdate()
        {
            if (_builder == null) return;

            // --- TimingDiag: FixedUpdate = 物理が見るボーン姿勢 (このフレームのPush前)。 ---
            if (_diagLeft > 0 && _diagTr != null && _diagFixedCount == 0) _diagFixedPos = _diagTr.position;

            // 1. ボーン追従剛体に目標姿勢を渡す (物理前)。
            PushBonesToKinematic();

            // 2. 物理ステップ。
            _builder.World.StepSimulation(Time.fixedDeltaTime);
            if (_diagLeft > 0) { _diagFixedCount++; _diagDt = Time.fixedDeltaTime; _diagSteps += _builder.World.LastStepsRun; }

            // 3. 物理剛体 -> ボーンへ反映 (物理後)。
            PullPhysicsToBones();
        }

        // 起動直後の数フレームだけ、アニメが確定させた「フレーム0姿勢」に対して物理を再整合する。
        // Animator は Update と LateUpdate の間で姿勢を書くため、ここでは posed 骨格が反映済み。
        // FK-rest リセットで動的剛体(スカート/髪)を posed 骨格の周りへ置き直し、バインド→フレーム0の
        // 瞬間移動で生じる脚への深い貫入(突き抜け)平衡を回避する。指定フレーム経過後はライブ物理へ。
        void LateUpdate()
        {

            // --- TimingDiag: LateUpdate = Animator(Normal)適用後。表示に最も近い姿勢。 ---
            if (_diagLeft > 0 && _diagTr != null)
            {
                var late = _diagTr.position;
                float dFL = _diagFixedCount > 0 ? Dist(_diagFixedPos, late) : -1f;
                float dUL = Dist(_diagUpdatePos, late);
                if (dFL >= 0) _diagDFL.Add(dFL);
                _diagDUL.Add(dUL);
                if (_diagLeft > 110 || dFL > 0.02f) // 最初の10フレームは全部、以降は大きい遅れのみ出力
                    Debug.Log($"[TimingDiag] F{Time.frameCount} fixedCalls={_diagFixedCount} steps={_diagSteps} dt={_diagDt:F4} | bone Fixed=({_diagFixedPos.x:F3},{_diagFixedPos.y:F3},{_diagFixedPos.z:F3}) Late=({late.x:F3},{late.y:F3},{late.z:F3}) dFL={dFL:F4} dUL={dUL:F4}");
                _diagFixedCount = 0; _diagSteps = 0;
                if (--_diagLeft == 0)
                {
                    _diagDFL.Sort(); _diagDUL.Sort();
                    float MedOf(List<float> v) => v.Count > 0 ? v[v.Count / 2] : 0f;
                    float MaxOf(List<float> v) => v.Count > 0 ? v[v.Count - 1] : 0f;
                    Debug.Log($"[TimingDiag] 要約: dFL(物理が見た姿勢と表示姿勢の差) 中央={MedOf(_diagDFL):F4} 最大={MaxOf(_diagDFL):F4} | dUL(Update vs Late=Animator書込タイミング) 中央={MedOf(_diagDUL):F4} 最大={MaxOf(_diagDUL):F4}"
                        + " 判定: dFL,dULとも大 → AnimatorはUpdate後に書く=物理は1フレーム古い体を見ている(遅れ確定)。dFL≈0 → 遅れなし=別因。");
                }
            }

            if (_startupResetCountdown <= 0 || _builder == null) return;
            ResetPhysicsToBones();
            _startupResetCountdown--;
        }

        private void PushBonesToKinematic()
        {
            // 駆動式は共通ヘルパに集約 (2026-08-09 hairfid誤配置事故の再発防止)。
            // 旧実装は未解決ボーンで Identity フォールバック=原点へテレポートし得た。ヘルパは null=前回維持で安全。
            _builder.ApplyKinematicTargets(BoneWorldOrNull);
        }

        private void PullPhysicsToBones()
        {
            // 補正層再現: 位置=親チェーン再構成 / 回転=物理 (共通ヘルパ)。
            RigidTransform?[] aligned = AlignBonePositions ? _builder.ComputeAlignedBonePoses(BoneWorldOrNull, AlignRotClampAlpha) : null;
            foreach (var link in _builder.BoneLinks)
            {
                if (link.Mode == PhysicsMode.BoneFollow) continue;
                if (link.BoneIndex < 0 || _boneTransforms == null ||
                    link.BoneIndex >= _boneTransforms.Length) continue;
                var tr = _boneTransforms[link.BoneIndex];
                if (tr == null) continue;

                // body = bone * offset  ->  bone = body * offset^-1
                var boneWorld = (aligned != null && aligned[link.BoneIndex].HasValue)
                    ? aligned[link.BoneIndex].Value
                    : link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();
                tr.position = MmdToUnityPos(boneWorld.Origin);
                tr.rotation = MmdToUnityRot(boneWorld.Rotation);
            }
        }

        private RigidTransform BoneWorld(int boneIndex)
        {
            if (boneIndex < 0 || _boneTransforms == null ||
                boneIndex >= _boneTransforms.Length || _boneTransforms[boneIndex] == null)
                return RigidTransform.Identity;
            var tr = _boneTransforms[boneIndex];
            return new RigidTransform(UnityToMmdRot(tr.rotation), UnityToMmdPos(tr.position));
        }

        // --- 座標変換 (MMD ネイティブ <-> Unity, 単位スケールのみ。Z反転なし) ---
        // メッシュ/スケルトンは mmd2gltf(ReverseZ) → UniGLTF(ReverseZ) の二重ReverseZが相殺し、
        // Unityボーンは既にPMXネイティブ座標値になっている。物理剛体もGlbPhysicsReaderがraw PMXで構築。
        // 従って境界は「単位スケールのみ」の真の等長変換にする(以前の3回目Z反転=鏡映バグを除去)。
        // 実測(DumpZ)でX-Z掌性の符号反転=鏡映を確認済み(PMX+1.583 vs U2M-1.583)。
        public Vector3 MmdToUnityPos(Vec3 v) => new(v.x * UnitScale, v.y * UnitScale, v.z * UnitScale);
        public Vec3 UnityToMmdPos(Vector3 v) => new(v.x / UnitScale, v.y / UnitScale, v.z / UnitScale);
        public static Quaternion MmdToUnityRot(Quat q) => new(q.x, q.y, q.z, q.w);
        public static Quat UnityToMmdRot(Quaternion q) => new(q.x, q.y, q.z, q.w);

        void OnDrawGizmos()
        {
            if (!DrawGizmos || _builder == null) return;
            foreach (var body in _builder.Bodies)
            {
                Gizmos.color = body.Mode == PhysicsMode.BoneFollow
                    ? Color.cyan
                    : (body.Mode == PhysicsMode.Dynamic ? Color.green : Color.yellow);
                var p = MmdToUnityPos(body.WorldTransform.Origin);
                var rot = MmdToUnityRot(body.WorldTransform.Rotation);
                DrawShape(body.Shape, p, rot);
            }
        }

        private void DrawShape(CollisionShape shape, Vector3 pos, Quaternion rot)
        {
            var m = Gizmos.matrix;
            // 形状サイズは PMX ネイティブ単位なので UnitScale を掛けて位置と揃える。
            Gizmos.matrix = UnityEngine.Matrix4x4.TRS(pos, rot, Vector3.one * UnitScale);
            switch (shape)
            {
                case SphereShape s:
                    Gizmos.DrawWireSphere(Vector3.zero, s.Radius);
                    break;
                case BoxShape b:
                    Gizmos.DrawWireCube(Vector3.zero, new Vector3(b.HalfExtents.x, b.HalfExtents.y, b.HalfExtents.z) * 2f);
                    break;
                case CapsuleShape c:
                    Gizmos.DrawWireSphere(new Vector3(0, c.HalfHeight, 0), c.Radius);
                    Gizmos.DrawWireSphere(new Vector3(0, -c.HalfHeight, 0), c.Radius);
                    break;
            }
            Gizmos.matrix = m;
        }
    }
}
