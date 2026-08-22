Set-Location C:\mytask2\unity-bullet-physics\tools\diagnostics\bulletref
Write-Output "  == 純Bullet (bulletref) =="
foreach ($d in (Get-ChildItem "C:\mytask2\unity-bullet-physics\tools\fricprobe\net_fric_*" -Directory | Sort-Object Name)) {
  $out = "C:\mytask2\unity-bullet-physics\tools\fricprobe\bul_" + $d.Name
  New-Item -ItemType Directory -Force $out | Out-Null
  & ".\bulletref.exe" --net ($d.FullName + "\net.txt") --out $out --frames 120 --substeps 2 --iters 10 --erp 0.2 --keepmargin 2>&1 | Out-Null
}
Write-Output done
