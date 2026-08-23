Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\tools\fricprobe\fric_L_tan030.pmx")
[Environment]::SetEnvironmentVariable("FRAMES","121")
foreach ($fm in @("1","0")) {
  [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
  [Environment]::SetEnvironmentVariable("TRAJ_OUT",("traj_engine_fm" + $fm + ".csv"))
  & ".\bin\Release\net9.0\FricProbe.exe" 2>&1 | Select-String "^  box"
}
