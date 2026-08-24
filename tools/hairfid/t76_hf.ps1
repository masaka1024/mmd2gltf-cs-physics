Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
# タスク76: FRICMUL(積=正しい mu) を入れたうえで FRICALIGN の A/B。
#   FRICALIGN=1 は摩擦の接線方向を Bullet と同じ「接線相対速度に整列・1方向」にする。
#   既定(=0) は軸任意の直交2方向で、bulletref 比で方向が 46〜51度ずれ、
#   |t| が mu*N の最大 1.414倍 (箱型の角) まで出ていた。
foreach ($fa in @("0","1")) {
  foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL")) { [Environment]::SetEnvironmentVariable($k,$null) }
  foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST")) { [Environment]::SetEnvironmentVariable($k,"1") }
  [Environment]::SetEnvironmentVariable("LEVER","1")
  [Environment]::SetEnvironmentVariable("FRICMUL","1")
  [Environment]::SetEnvironmentVariable("FRICALIGN",$fa)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  Write-Output ("===== FRICMUL=1  FRICALIGN=" + $fa)
  $o
}
foreach ($k in @("FRICMUL","FRICALIGN")) { [Environment]::SetEnvironmentVariable($k,$null) }
$o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
Write-Output "===== 出荷既定 (FRICMUL=0 FRICALIGN=0)"
$o
