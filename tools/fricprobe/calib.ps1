Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
Write-Output "  == 当エンジン =="
foreach ($fm in @("0","1")) {
  Write-Output ("  -- FRICMUL=" + $fm + ("  (0=幾何平均 / 1=積)"))
  foreach ($p in (Get-ChildItem *.pmx | Sort-Object Name)) {
    foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT")) { [Environment]::SetEnvironmentVariable($k,$null) }
    [Environment]::SetEnvironmentVariable("FRICMUL",$fm)
    [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$p.FullName)
    [Environment]::SetEnvironmentVariable("FRAMES","120")
    $o = & ".\bin\Release\net9.0\FricProbe.exe" 2>&1
    $line = ($o | Select-String "^  box") -replace '\s+',' '
    Write-Output ("     " + $p.BaseName.PadRight(16) + " " + $line)
  }
}
