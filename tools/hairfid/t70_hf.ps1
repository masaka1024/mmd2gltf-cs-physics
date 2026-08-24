Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
foreach ($sd in @("0","1")) { foreach ($bm in @("0.2","0")) {
  foreach ($k in @("CTHRESH","ROTEXP","CMARGIN","MIXAXES","LEVER","CNBF","JOINTS_FIRST","CRHS","CMAN","LIMGATE","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR")) { [Environment]::SetEnvironmentVariable($k,$null) }
  [Environment]::SetEnvironmentVariable("SYMDIST",$sd)
  [Environment]::SetEnvironmentVariable("BAUM",$bm)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  Write-Output ("===== SYMDIST=" + $sd + " x BAUM=" + $bm)
  $o | Select-String "自前 深貫入|MMD 深貫入|傾き|12窓比の中央値" | Select-Object -First 4
  $o | Select-String "^\[髪×体 貫入\] ペア別" -Context 0,3
}}
