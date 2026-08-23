# fric_L_tan030 を 180F(6秒) 相当で当エンジン/純Bullet 側にも走らせておく (比較の相手)
Set-Location C:\mytask2\unity-bullet-physics\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("FRICMUL","1")
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","C:\mytask2\unity-bullet-physics\tools\fricprobe\fric_L_tan030.pmx")
[Environment]::SetEnvironmentVariable("NETDUMP","1"); [Environment]::SetEnvironmentVariable("KEEPBODY","box")
[Environment]::SetEnvironmentVariable("EXTRABODIES","slope,box")
[Environment]::SetEnvironmentVariable("SUBSTEPS","2"); [Environment]::SetEnvironmentVariable("ITERS","10"); [Environment]::SetEnvironmentVariable("FRAMES","361")
[Environment]::SetEnvironmentVariable("OUTDIR","out_fm1_fric_L_tan030")
& "C:\mytask2\unity-bullet-physics\tools\diagnostics\restosc\bin\Release\net9.0\RestOsc.exe" 2>&1 | Out-Null
# NetDump は網の外の剛体を落とすので net.txt を直してから bulletref
$p = "out_fm1_fric_L_tan030\net.txt"
(Get-Content $p -Encoding utf8) | ForEach-Object {
  if ($_ -like "body * name=box *") { $_ -replace " mode=0", " mode=1" -replace " mask=0", " mask=65535" } else { $_ }
} | Set-Content $p -Encoding utf8
Set-Location C:\mytask2\unity-bullet-physics\tools\diagnostics\bulletref
New-Item -ItemType Directory -Force "C:\mytask2\unity-bullet-physics\tools\fricprobe\bul_net_fric_L_tan030" | Out-Null
& ".\bulletref.exe" --net "C:\mytask2\unity-bullet-physics\tools\fricprobe\out_fm1_fric_L_tan030\net.txt" --out "C:\mytask2\unity-bullet-physics\tools\fricprobe\bul_net_fric_L_tan030" --frames 361 --substeps 2 --iters 10 --erp 0.2 --keepmargin 2>&1 | Select-String "bodies="
Write-Output done
