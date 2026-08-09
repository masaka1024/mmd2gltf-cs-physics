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
        public Transform[] GetComponentsInChildren<T>() => new Transform[0];
    }

    public class Transform : Component
    {
        public string name;
        public Vector3 position;
        public Quaternion rotation;
        // 診断(DumpZHistory)がシーン配置の確認に使う。ヘッドレスでは実体が無いので既定値を返すだけ。
        public Vector3 eulerAngles => new Vector3(0, 0, 0);
        public Vector3 lossyScale => new Vector3(1, 1, 1);
    }

    public class MonoBehaviour : Component { }

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
