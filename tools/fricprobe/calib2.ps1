Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
Write-Output "  == 第2ラウンドの較正 (対B: f0=f1=0.5) =="
foreach ($fm in @("1","0")) {
  Write-Output ("  -- FRICMUL=" + $fm + "  (1=積 μ=0.25 / 0=幾何平均 μ=0.5)")
  foreach ($p in (Get-ChildItem fric_L_*.pmx | Sort-Object Name -Descending)) {
    foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT")) { [Environment]::SetEnvironmentVariable($k,$null) }
    [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
    [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$p.FullName)
    [Environment]::SetEnvironmentVariable("FRAMES","120")
    $o = & ".\bin\Release\net9.0\FricProbe.exe" 2>&1
    $line = ($o | Select-String "^  box") -replace '\s+',' '
    Write-Output ("     " + $p.BaseName.PadRight(16) + " " + $line)
  }
}
