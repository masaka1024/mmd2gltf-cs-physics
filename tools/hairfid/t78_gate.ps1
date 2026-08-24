Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
# タスク78: 腕長ゲートの効き。閾値は (|rA|+|rB|) 比。
#   受け入れ条件: (1) 既定条件で発動0行 = v1 とビット同値  (2) FRICMUL=1 の発散が消える
$cases = @(
  @{n="既定 v1 (ゲート無)";  fm="0"; g=$null},
  @{n="既定 v1 + ゲート5";   fm="0"; g="5"},
  @{n="既定 v1 + ゲート3";   fm="0"; g="3"},
  @{n="FRICMUL=1 ゲート無";  fm="1"; g=$null},
  @{n="FRICMUL=1 + ゲート5"; fm="1"; g="5"},
  @{n="FRICMUL=1 + ゲート3"; fm="1"; g="3"}
)
foreach ($c in $cases) {
  foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL","FRICALIGN","RUNAWAY","RUNAWAY_BONE","RUNAWAY_JOINT","LEVERGATE")) { [Environment]::SetEnvironmentVariable($k,$null) }
  foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST")) { [Environment]::SetEnvironmentVariable($k,"1") }
  [Environment]::SetEnvironmentVariable("LEVER","1")
  [Environment]::SetEnvironmentVariable("FRICMUL",$c.fm)
  if ($c.g) { [Environment]::SetEnvironmentVariable("LEVERGATE",$c.g) }
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  $p = [regex]::Match((($o | Select-String "位置差\(u\)") -join " "), "中央=([0-9.]+)/p90=([0-9.]+)/最大=([0-9.]+)")
  $h = [regex]::Match((($o | Select-String "腕長ゲート") -join " "), "発動行数=([0-9]+)").Groups[1].Value
  Write-Output ("  {0,-22} 中央={1,-7} p90={2,-7} 最大={3,-12} ゲート発動={4}" -f $c.n,$p.Groups[1].Value,$p.Groups[2].Value,$p.Groups[3].Value,$h)
}
