Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
foreach ($sd in @("0","1")) {
  foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM")) { [Environment]::SetEnvironmentVariable($k,$null) }
  foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","CSET","NORMFIRST","CRHS","CMAN","LIMGATE")) { [Environment]::SetEnvironmentVariable($k,"1") }
  [Environment]::SetEnvironmentVariable("LEVER","1")
  [Environment]::SetEnvironmentVariable("SYMDIST",$sd)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  Write-Output ("===== フルスタック SYMDIST=" + $sd)
  $o | Select-String "実効\] hairfid" | Select-Object -First 1
  $o | Select-String "自前 深貫入|MMD 深貫入|曲げ角" | Select-Object -First 3
}
