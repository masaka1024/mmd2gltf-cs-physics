// ===========================================================================
// 物理バックエンドのトグル並立: PhysX版(既存 mmd2gltf インポーターが付ける
// Rigidbody + ConfigurableJoint)と、自作エンジン版(MmdPhysicsBehaviour)を排他切替する。
//
// 設計方針:
//   - Unity 組込み型 (Rigidbody / ConfigurableJoint) だけを操作するため、
//     既存インポーターのソースは一切変更しない (importer への依存も持たない)。
//   - Custom: 自作エンジンを有効化し、PhysX 剛体を「パーク」(isKinematic=true,
//     detectCollisions=false) して同じボーンを奪い合わないようにする。ConfigurableJoint も無効化。
//   - PhysX: 自作エンジンを無効化し、PhysX 剛体/Joint を元の状態へ復帰する。
//   - どちらも同時に同じボーン Transform を動かさない (排他)。
//   - 物理以外 (lilToon マテリアル / スキンバインディング / ベイク再生) には触れない。
//
// ※ Unity の PhysX 型 (Rigidbody/ConfigurableJoint) を使うため、UnityEngine 最小シムの
//   検証ハーネスからは除外し (#if)、Unity 実機でのみコンパイルされる。
// ===========================================================================
#if UNITY_2019_1_OR_NEWER
using System.Collections.Generic;
using UnityEngine;

namespace BulletPhysics.Unity
{
    [DisallowMultipleComponent]
    public sealed class MmdPhysicsBackendSwitch : MonoBehaviour
    {
        public enum Backend { Custom, PhysX }

        [Tooltip("Custom=自作エンジン / PhysX=既存インポーターの Rigidbody+ConfigurableJoint")]
        public Backend Mode = Backend.Custom;

        [Tooltip("自作エンジン (未指定ならこの GameObject 以下から自動検出)")]
        public MmdPhysicsBehaviour customEngine;

        private Rigidbody[] _rbs;
        private bool[] _origKinematic;
        private bool[] _origDetectCollisions;
        private bool _snapped;

        void Awake() { Snapshot(); ApplyBackend(); }

        void OnValidate() { if (Application.isPlaying && _snapped) ApplyBackend(); }

        // PhysX コンポーネントの初期状態を控える (元の isKinematic を PhysX 復帰で使う)。
        private void Snapshot()
        {
            _rbs = GetComponentsInChildren<Rigidbody>(true);
            _origKinematic = new bool[_rbs.Length];
            _origDetectCollisions = new bool[_rbs.Length];
            for (int i = 0; i < _rbs.Length; i++)
            {
                _origKinematic[i] = _rbs[i].isKinematic;
                _origDetectCollisions[i] = _rbs[i].detectCollisions;
            }
            if (customEngine == null) customEngine = GetComponentInChildren<MmdPhysicsBehaviour>(true);
            _snapped = true;
        }

        [ContextMenu("Apply Backend (現在のModeを適用)")]
        public void ApplyBackend()
        {
            if (!_snapped) Snapshot();
            bool custom = Mode == Backend.Custom;

            // 自作エンジン: Custom で有効、PhysX で無効。
            if (customEngine != null)
            {
                customEngine.enabled = custom;
                // 切替直後にボーンが PhysX 側で動いていた場合に備え、Custom へ入る時は
                // 現在のボーン姿勢へ FK-rest 再整合する (未ロードなら内部で無視される)。
                if (custom) customEngine.ResetPhysicsToBones();
            }

            // PhysX 剛体: Custom ではパーク (kinematic/衝突無効)、PhysX では元状態へ復帰。
            for (int i = 0; i < _rbs.Length; i++)
            {
                var rb = _rbs[i];
                if (rb == null) continue;
                if (custom) { rb.isKinematic = true; rb.detectCollisions = false; }
                else { rb.isKinematic = _origKinematic[i]; rb.detectCollisions = _origDetectCollisions[i]; }
            }
            // ConfigurableJoint は Behaviour ではないため .enabled で無効化できない。
            // だが Custom では上のループで全 Rigidbody を isKinematic=true にパークしており、
            // kinematic なボディは Joint の拘束/ドライブで動かされない=Joint は自動的に無効(inert)になる。
            // PhysX へ戻すと isKinematic が元に戻り Joint も自然に復帰するため、個別操作は不要。
        }

        [ContextMenu("Use Custom (自作エンジン)")]
        public void UseCustom() { Mode = Backend.Custom; ApplyBackend(); }

        [ContextMenu("Use PhysX (既存インポーター)")]
        public void UsePhysX() { Mode = Backend.PhysX; ApplyBackend(); }
    }
}
#endif
