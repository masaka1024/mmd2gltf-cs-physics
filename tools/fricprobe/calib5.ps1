Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET","SLEEP","LINDAMP","ANGDAMP")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($n in @("fricT_B_tan030","fricT_B_tan100","fricT_B_tan120")) {
  foreach ($fm in @("1","0")) {
    [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
    [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",("C:\mytask2\unity-bullet-physics\tools\fricprobe\" + $n + ".pmx"))
    [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_" + $n + "_fm" + $fm + ".csv"))
    $o = & ".\bin\Release\net9.0\FricProbe.exe" 2>&1
    Write-Output ("  " + $n.PadRight(18) + " FRICMUL=" + $fm + "  " + (($o | Select-String "^  box") -replace '\s+',' '))
  }
}
