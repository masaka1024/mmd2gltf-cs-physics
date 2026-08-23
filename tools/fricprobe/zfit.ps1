Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET","SLEEP","ANGDAMP")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\tools\fricprobe\fric_Z_tan030.pmx")
[Environment]::SetEnvironmentVariable("FRICMUL","1"); [Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($d in @("0","0.05","0.07","0.10")) {
  [Environment]::SetEnvironmentVariable("LINDAMP",$d)
  [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_Z_d" + $d + ".csv"))
  & ".\bin\Release\net9.0\FricProbe.exe" 2>&1 | Out-Null
}
Write-Output done
