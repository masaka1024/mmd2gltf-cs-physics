Set-Location C:\mytask2\unity-bullet-physics\tools\bonecheck
# タスク78: drivedp。出荷既定 / +ゲート5 / FRICMUL=1+ゲート5。
$cfgs = @(@{n="出荷既定";fm=$null;g=$null}, @{n="+ゲート5";fm=$null;g="5"}, @{n="FRICMUL=1+ゲート5";fm="1";g="5"})
foreach ($c in $cfgs) {
  foreach ($k in @("CMARGIN","MODELS","MINNET","BONEDP","BODIES","REFFLOOR","FRAMES","SPRINGCLAMP","SPRINGMOTOR",
                   "NETDUMP","KEEPBODY","EXTRABODIES","INITSTATE","JOINTORDER","ROWTRACE","CONTACTCSV","OUTDIR",
                   "SMOKE","SLOP","MANSTATS","ITERS","SUBSTEPS","JOINTS_FIRST","NOCONTACT","ANCHORDUMP","REPLAY",
                   "OUT","MIXAXES","CNBF","BAUM","CSET","NORMFIRST","CPOOL","FRICALIGN","FRICMUL","LEVERGATE",
                   "JBETA","MAXCORR","DPPART","BONECSV_OUT","DPFRAMES_OUT")) { [Environment]::SetEnvironmentVariable($k,$null) }
  foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","CRHS","CMAN","LIMGATE","SYMDIST")) { [Environment]::SetEnvironmentVariable($k,"1") }
  [Environment]::SetEnvironmentVariable("LEVER","1")
  [Environment]::SetEnvironmentVariable("DRIVEDP","1"); [Environment]::SetEnvironmentVariable("AB","none")
  [Environment]::SetEnvironmentVariable("FRICMUL",$c.fm)
  [Environment]::SetEnvironmentVariable("LEVERGATE",$c.g)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_BONECSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  Write-Output ("===== " + $c.n)
  $o = & ".\bin\Release\net9.0\BoneCheck.exe" 2>&1
  $o
}
