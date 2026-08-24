# タスク78 採用後の確認: env を一切設定せずに走らせて、出荷既定が新構成になっているか。
Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL","FRICALIGN","FRICMUL","LEVERGATE","LEVERPROBE","RUNAWAY","RUNAWAY_BONE","RUNAWAY_JOINT","ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST","LEVER")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
[Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
$o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
$o | Select-String "実効\] hairfid|位置差\(u\)|腕長ゲート"
