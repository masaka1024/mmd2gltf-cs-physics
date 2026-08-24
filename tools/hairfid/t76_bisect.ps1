Set-Location C:\mytask2\unity-bullet-physics\tools\hairfid
# タスク76: FRICMUL=1 で モミアゲL-L1 のジョイントが指数的に緩む。どの要素が要るかを切り分ける。
$cases = @(
  @{n="基準 (v1+FRICMUL)"; e=@{}},
  @{n="LIMGATE=0";        e=@{LIMGATE="0"}},
  @{n="LEVER=0";          e=@{LEVER="0"}},
  @{n="LEVER=2";          e=@{LEVER="2"}},
  @{n="MAXCORR=1e9";      e=@{MAXCORR="1000000000"}},
  @{n="ANGCONV=0";        e=@{ANGCONV="0"}},
  @{n="AXES=0";           e=@{AXES="0"}},
  @{n="CMAN=0";           e=@{CMAN="0"}},
  @{n="ROTEXP=0";         e=@{ROTEXP="0"}},
  @{n="SUBSTEPS=2";       e=@{SUBSTEPS="2"}}
)
foreach ($c in $cases) {
  foreach ($k in @("CMARGIN","MIXAXES","CNBF","JOINTS_FIRST","SUBSTEPS","ITERS","SPLIT","JSPLIT","CWFAC","WARMFAC","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN","SPRINGMOTOR","BAUM","CSET","CPOOL","FRICALIGN","RUNAWAY","RUNAWAY_BONE")) { [Environment]::SetEnvironmentVariable($k,$null) }
  foreach ($k in @("ANGCONV","AXES","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST","FRICMUL")) { [Environment]::SetEnvironmentVariable($k,"1") }
  [Environment]::SetEnvironmentVariable("LEVER","1")
  foreach ($k in $c.e.Keys) { [Environment]::SetEnvironmentVariable($k,$c.e[$k]) }
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx")
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
  $m = ($o | Select-String "\[全体\] 位置差") -replace ".*最大=([0-9.]+).*",'$1'
  Write-Output ("  {0,-22} 位置差 最大 = {1}" -f $c.n, $m)
}
