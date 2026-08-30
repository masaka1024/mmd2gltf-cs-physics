# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
# fric_L_tan030 を 180F(6秒) 相当で当エンジン/純Bullet 側にも走らせておく (比較の相手)
Set-Location $REPO\tools\fricprobe
foreach ($k in @("FRICALIGN","CPOOL","NORMFIRST","OUT","CSET")) { [Environment]::SetEnvironmentVariable($k,$null) }
[Environment]::SetEnvironmentVariable("FRICMUL","1")
[Environment]::SetEnvironmentVariable("MMD_TEST_PMX","$REPO\tools\fricprobe\fric_L_tan030.pmx")
[Environment]::SetEnvironmentVariable("NETDUMP","1"); [Environment]::SetEnvironmentVariable("KEEPBODY","box")
[Environment]::SetEnvironmentVariable("EXTRABODIES","slope,box")
[Environment]::SetEnvironmentVariable("SUBSTEPS","2"); [Environment]::SetEnvironmentVariable("ITERS","10"); [Environment]::SetEnvironmentVariable("FRAMES","361")
[Environment]::SetEnvironmentVariable("OUTDIR","out_fm1_fric_L_tan030")
& "$REPO\tools\diagnostics\restosc\bin\Release\net9.0\RestOsc.exe" 2>&1 | Out-Null
# NetDump は網の外の剛体を落とすので net.txt を直してから bulletref
$p = "out_fm1_fric_L_tan030\net.txt"
(Get-Content $p -Encoding utf8) | ForEach-Object {
  if ($_ -like "body * name=box *") { $_ -replace " mode=0", " mode=1" -replace " mask=0", " mask=65535" } else { $_ }
} | Set-Content $p -Encoding utf8
Set-Location $REPO\tools\diagnostics\bulletref
New-Item -ItemType Directory -Force "$REPO\tools\fricprobe\bul_net_fric_L_tan030" | Out-Null
& ".\bulletref.exe" --net "$REPO\tools\fricprobe\out_fm1_fric_L_tan030\net.txt" --out "$REPO\tools\fricprobe\bul_net_fric_L_tan030" --frames 361 --substeps 2 --iters 10 --erp 0.2 --keepmargin 2>&1 | Select-String "bodies="
Write-Output done
