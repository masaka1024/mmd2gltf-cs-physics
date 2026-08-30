# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $REPO\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET","SLEEP","LINDAMP","ANGDAMP")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($n in @("fric_S_tan150","fric_Z_tan030")) {
  foreach ($fm in @("1","0")) {
    [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
    [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",("$REPO\tools\fricprobe\" + $n + ".pmx"))
    [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_" + $n + "_fm" + $fm + ".csv"))
    $o = & ".\bin\Release\net9.0\FricProbe.exe" 2>&1
    Write-Output ("  " + $n + " FRICMUL=" + $fm + "  " + (($o | Select-String "^  box") -replace '\s+',' '))
  }
}
