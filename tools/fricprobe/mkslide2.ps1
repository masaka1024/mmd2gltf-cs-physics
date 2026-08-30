# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $REPO\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","$REPO\tools\fricprobe\fric_L_tan030.pmx")
[Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($fm in @("1","0")) {
  [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
  [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_engine_fm" + $fm + ".csv"))
  & ".\bin\Release\net9.0\FricProbe.exe" 2>&1 | Select-String "^  box"
}
