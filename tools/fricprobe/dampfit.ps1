Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET","SLEEP","ANGDAMP")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\tools\fricprobe\fric_L_tan030.pmx")
[Environment]::SetEnvironmentVariable("FRICMUL","1"); [Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($d in @("0","0.5","0.9","0.99")) {
  [Environment]::SetEnvironmentVariable("LINDAMP",$d)
  [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_damp" + $d + ".csv"))
  & ".\bin\Release\net9.0\FricProbe.exe" 2>&1 | Out-Null
}
Write-Output done
