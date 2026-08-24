Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
# タスク76: FRICMUL=1 で もみあげL が f4167 から発散する。その窓を毎フレーム覗く。
foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL","FRICALIGN")) { [Environment]::SetEnvironmentVariable($k,$null) }
foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST")) { [Environment]::SetEnvironmentVariable($k,"1") }
[Environment]::SetEnvironmentVariable("LEVER","1")
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
[Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
[Environment]::SetEnvironmentVariable("RUNAWAY","4128,4155")
[Environment]::SetEnvironmentVariable("RUNAWAY_BONE","モミアゲL")
foreach ($fm in @("1")) {
  [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
  Write-Output ("===== FRICMUL=" + $fm)
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  $o | Select-String "^\[RW"
}
