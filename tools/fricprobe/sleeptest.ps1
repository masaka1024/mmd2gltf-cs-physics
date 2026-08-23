Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET","SLEEP_LIN","SLEEP_ANG","SLEEP_T")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\tools\fricprobe\fric_L_tan030.pmx")
[Environment]::SetEnvironmentVariable("FRICMUL","1"); [Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($s in @("0","1")) {
  [Environment]::SetEnvironmentVariable("SLEEP",$s)
  [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_sleep" + $s + ".csv"))
  $o = & ".\bin\Release\net9.0\FricProbe.exe" 2>&1
  Write-Output ("-- SLEEP=" + $s); $o | Select-String "実効|^  box"
}
