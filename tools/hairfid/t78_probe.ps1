Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
# タスク78: 腕長比 |anchorB-anchorA| / (|rA|+|rB|) の分布を実測して閾値を決める。
foreach ($fm in @("0","1")) {
  foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL","FRICALIGN","RUNAWAY","RUNAWAY_BONE","RUNAWAY_JOINT","LEVERGATE")) { [Environment]::SetEnvironmentVariable($k,$null) }
  foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST","LEVERPROBE")) { [Environment]::SetEnvironmentVariable($k,"1") }
  [Environment]::SetEnvironmentVariable("LEVER","1")
  [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  Write-Output ("===== FRICMUL=" + $fm)
  $o | Select-String "腕長比の分布"
}
