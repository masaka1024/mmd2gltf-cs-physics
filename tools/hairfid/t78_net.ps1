# タスク78: 採用の正味の効果。★NORMFIRST は出荷既定に入っていないので立てない。
Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
$cfgs = @(@{n="旧出荷既定";fm="0";g="0"}, @{n="ゲートのみ";fm="0";g="5"}, @{n="新出荷既定";fm=$null;g=$null})
foreach ($c in $cfgs) {
  foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL","FRICALIGN","FRICMUL","LEVERGATE","LEVERPROBE","RUNAWAY","RUNAWAY_BONE","RUNAWAY_JOINT","ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST","LEVER")) { [Environment]::SetEnvironmentVariable($k,$null) }
  [Environment]::SetEnvironmentVariable("FRICMUL",$c.fm)
  [Environment]::SetEnvironmentVariable("LEVERGATE",$c.g)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  $p = [regex]::Match((($o | Select-String "位置差\(u\)") -join " "), "中央=([0-9.]+)/p90=([0-9.]+)/最大=([0-9.]+)")
  $a = [regex]::Match((($o | Select-String "位置差\(u\)") -join " "), "角度差.*中央=([0-9.]+)/p90=([0-9.]+)")
  $h = [regex]::Match((($o | Select-String "腕長ゲート") -join " "), "発動行数=([0-9]+)").Groups[1].Value
  $d = (($o | Select-String "自前 深貫入") -join " ")
  Write-Output ("  {0,-12} 位置差 中央={1,-7} p90={2,-7} 最大={3,-9} 角度 中央={4,-6} p90={5,-6} ゲート={6}" -f $c.n,$p.Groups[1].Value,$p.Groups[2].Value,$p.Groups[3].Value,$a.Groups[1].Value,$a.Groups[2].Value,$h)
  Write-Output ("               " + $d.Trim())
}
