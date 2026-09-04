// 最小 UnityEngine シム (Unity 非依存のコンパイル/検証専用。Unity 実行時には使用しない)。
// Assets/Scripts が参照する UnityEngine の型だけを最小限で満たす。
using System;

namespace UnityEngine
{
    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static Vector3 one => new Vector3(1, 1, 1);
        public static Vector3 zero => new Vector3(0, 0, 0);
        public static Vector3 operator *(Vector3 a, float s) => new Vector3(a.x * s, a.y * s, a.z * s);
        public static Vector3 operator +(Vector3 a, Vector3 b) => new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    public struct Quaternion
    {
        public float x, y, z, w;
        public Quaternion(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
    }

    public struct Matrix4x4
    {
        public float m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33;
        public static Matrix4x4 TRS(Vector3 p, Quaternion q, Vector3 s) => new Matrix4x4();
    }

    public struct Color
    {
        public float r, g, b, a;
        public Color(float r, float g, float b, float a) { this.r = r; this.g = g; this.b = b; this.a = a; }
        public static Color cyan => new Color();
        public static Color green => new Color();
        public static Color yellow => new Color();
        public static Color magenta => new Color();
        public static Color red => new Color();
        public static Color white => new Color();
    }

    public class Component
    {
        public string name;
        public Transform[] GetComponentsInChildren<T>() => new Transform[0];
        // 単体取得は「見つからない」を返すだけでよい (ヘッドレスにはシーンが無い)。
        public T GetComponentInParent<T>() where T : class => null;
        public T GetComponentInChildren<T>() where T : class => null;
        // includeInactive 付きオーバーロード (非アクティブな GameObject も探索対象にする)。
        public T GetComponentInParent<T>(bool includeInactive) where T : class => null;
        public T GetComponentInChildren<T>(bool includeInactive) where T : class => null;
    }

    // Animator.updateMode の切り替え (MmdPhysicsBehaviour.AlignAnimatorUpdateMode) が参照する。
    // ★Unity 2023.1 で AnimatePhysics → Fixed へ改名された。当プロジェクトは Unity 6 なので Fixed。
    public enum AnimatorUpdateMode { Normal, Fixed, UnscaledTime }

    public class Animator : Component
    {
        public AnimatorUpdateMode updateMode;
    }

    // レガシー Animation コンポーネント。★列挙型も値の名前も Animator 側とは別物。
    public enum AnimationUpdateMode { Normal, Fixed }   // ★実DLLのメタデータ準拠。Animator 側とは別物

    public class Animation : Component
    {
        public AnimationUpdateMode updateMode;
    }

    public class Transform : Component
    {
        public Vector3 position;
        public Quaternion rotation;
        // 診断(DumpZHistory)がシーン配置の確認に使う。ヘッドレスでは実体が無いので既定値を返すだけ。
        public Vector3 eulerAngles => new Vector3(0, 0, 0);
        public Vector3 lossyScale => new Vector3(1, 1, 1);
        // 取り込み経路の検証(CheckImportConvention)がモデルの配置を除去するのに使う。
        // ヘッドレスでは ModelRoot が無く検証も走らないので、恒等を返すだけでよい。
        public Vector3 InverseTransformPoint(Vector3 p) => p;
    }

    public class MonoBehaviour : Component { }

    // ビルド同梱の物理データ (.mmdphys.bytes) を読む経路が参照する。
    public class TextAsset
    {
        public string name;
        public byte[] bytes;
    }

    public static class Gizmos
    {
        public static Color color;
        public static Matrix4x4 matrix;
        public static void DrawWireSphere(Vector3 c, float r) { }
        public static void DrawWireCube(Vector3 c, Vector3 s) { }
        public static void DrawSphere(Vector3 c, float r) { }
        public static void DrawLine(Vector3 a, Vector3 b) { }
    }

    public static class Time
    {
        public static float fixedDeltaTime = 1f / 60f;
        public static float deltaTime = 1f / 60f;
        // 重いフレームでの FixedUpdate 追いつき上限 (MmdPhysicsBehaviour.LimitCatchUp が下げる)。
        public static float maximumDeltaTime = 1f / 3f;
        public static int frameCount = 0;
    }

    public static class Debug
    {
        public static void Log(object m) { }
        public static void LogWarning(object m) { }
        public static void LogError(object m) { }
        // context 付きオーバーロード (Unity ではログをクリックして対象を選択できる)。
        public static void Log(object m, object context) { }
        public static void LogWarning(object m, object context) { }
        public static void LogError(object m, object context) { }
    }

    public class TooltipAttribute : Attribute { public TooltipAttribute(string s) { } }
    public class HeaderAttribute : Attribute { public HeaderAttribute(string s) { } }
    public class ContextMenuAttribute : Attribute { public ContextMenuAttribute(string s) { } }
    public class RangeAttribute : Attribute { public RangeAttribute(float min, float max) { } }
}
